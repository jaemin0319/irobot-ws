"""FR3 random reach -> home recovery loop (runs until the user stops it).

  --mode random (default): unlimited cycles
  --mode fixed           : one cycle with config.FIXED_TARGET (verification only), then exit 0
  --mode view            : HOME_CHECK, then hold home until stopped
Exit codes: 0 = user stop / fixed-mode completed, 1 = FAILED.
"""
import argparse
import json
import logging
import logging.handlers
import math
import os
import shutil
import signal
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from . import config as C
from .core import Quintic, Robot, duration, generate_target, check_path


class Stop:
    def __init__(self):
        self.flag, self.reason, self.count = False, "", 0

    def __call__(self):
        return self.flag

    def request(self, reason):
        self.count += 1
        if self.count >= 2 and reason.startswith("SIG"):
            sys.exit(130)  # second Ctrl+C: immediate exit
        self.flag, self.reason = True, reason


class Failed(Exception):
    def __init__(self, cause, **info):
        super().__init__(cause)
        self.cause, self.info = cause, info


def prune_logs():
    C.LOG_DIR.mkdir(exist_ok=True)
    runs = sorted(C.LOG_DIR.glob("run_*.log*"), key=lambda p: p.stat().st_mtime, reverse=True)
    total = 0
    for p in runs:
        total += p.stat().st_size
        if total > C.RUN_LOGS_TOTAL - C.RUN_LOG_BYTES * (C.RUN_LOG_BACKUPS + 1):
            p.unlink()
    fails = sorted(C.LOG_DIR.glob("failure_*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    for p in fails[C.FAILURE_KEEP:]:
        p.unlink()


def setup_log(stamp):
    prune_logs()
    lg = logging.getLogger("irobot")
    lg.setLevel(logging.INFO)
    fh = logging.handlers.RotatingFileHandler(C.LOG_DIR / f"run_{stamp}.log",
                                              maxBytes=C.RUN_LOG_BYTES, backupCount=C.RUN_LOG_BACKUPS)
    fmt = logging.Formatter("%(asctime)s %(message)s")
    fh.setFormatter(fmt)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    lg.addHandler(fh)
    lg.addHandler(sh)
    return lg


class Sim:
    def __init__(self, args):
        self.args = args
        self.stop = Stop()
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_ = setup_log(self.stamp)
        self.rob = Robot()
        self.m = self.rob.m
        self.d = mujoco.MjData(self.m)
        self.viewer = None
        self.cycle = 0
        self.phase = "INIT_LOAD"
        self.target = None
        self.q_goal = None
        self.phase_t0 = 0.0
        self.wall_t0 = time.monotonic()
        self.last_disk = -1e9
        self.sat_steps = 0
        self.warn0 = 0
        self.shadow = None
        self.last_progress = (time.monotonic(), 0.0)

    # ---------- logging ----------
    def ev(self, name, **kw):
        kw = {k: (np.round(v, 5).tolist() if isinstance(v, np.ndarray) else v) for k, v in kw.items()}
        self.log_.info(json.dumps({"ev": name, "cycle": self.cycle, "phase": self.phase,
                                   "sim_t": round(self.d.time, 4), **kw}, default=str))

    def status(self, text):
        if self.viewer is not None:
            try:
                self.viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "irobot", text))
            except Exception:
                pass

    # ---------- stepping with runtime checks ----------
    def external_change(self):
        """Detect state written outside our controller (viewer reset, joint sliders, perturbation)."""
        if self.shadow is None:
            return None
        t, qp, qv = self.shadow
        if self.d.time < t:
            return "sim_time_decreased (viewer reset?)"
        if not (np.array_equal(self.d.qpos, qp) and np.array_equal(self.d.qvel, qv)):
            return "qpos/qvel changed outside controller"
        if np.any(self.d.xfrc_applied != 0) or np.any(self.d.qfrc_applied != 0):
            return "external force applied (viewer perturbation)"
        return None

    def runtime_checks(self, q_des=None):
        d, rob = self.d, self.rob
        if not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel)) and np.all(np.isfinite(d.qacc))):
            raise Failed("numerical_nan")
        w = int(sum(d.warning[i].number for i in range(len(d.warning))))
        if w > self.warn0:
            raise Failed("mujoco_warning", warnings=[int(d.warning[i].number) for i in range(len(d.warning))])
        q = rob.q(d)
        if np.any(q < rob.lo - 0.01) or np.any(q > rob.hi + 0.01):
            raise Failed("joint_limit_violation")
        if d.ncon > 0:
            c = d.contact[0]
            raise Failed("unexpected_contact", pair=[self.m.body(self.m.geom_bodyid[c.geom1]).name,
                                                     self.m.body(self.m.geom_bodyid[c.geom2]).name])
        if q_des is not None and np.max(np.abs(q_des - q)) > C.TRACK_ERR_MAX:
            raise Failed("tracking_error", track_err=float(np.max(np.abs(q_des - q))))

    def step(self, q_des, qd_des, qdd_des):
        """One physics step under control, real-time paced, viewer synced ~60 Hz."""
        if self.stop():
            raise KeyboardInterrupt
        ext = self.external_change()
        if ext:
            raise Failed("external_state_change", detail=ext)
        _, sat = self.rob.control(self.d, qd_des, qdd_des, q_des)
        self.sat_steps += sat
        mujoco.mj_step(self.m, self.d)
        self.runtime_checks(q_des)
        self.shadow = (self.d.time, self.d.qpos.copy(), self.d.qvel.copy())
        now = time.monotonic()
        self.last_progress = (now, self.d.time)
        if now - self.last_disk > C.DISK_CHECK_WALL:
            self.last_disk = now
            free = shutil.disk_usage(C.WS).free / 1e6
            if free < C.DISK_MIN_MB:
                raise Failed("disk_low", free_mb=round(free))
        if self.viewer is not None:
            if not self.viewer.is_running():
                self.stop.request("window_closed")
                raise KeyboardInterrupt
            lag = self.d.time - (now - self.sim_wall0)
            if lag > 0:
                time.sleep(lag)
            if now - self.last_sync > 1 / 60:
                self.viewer.sync()
                self.last_sync = now
                ext = self.external_change()
                if ext:
                    raise Failed("external_state_change", detail=ext)

    def resync_clock(self):
        """After a non-stepping computation (GEN_TARGET): keep real-time pacing without catch-up."""
        self.sim_wall0 = time.monotonic() - self.d.time

    # ---------- phases ----------
    def errors(self):
        p, z = self.rob.tcp(self.d)
        return p, math.acos(float(np.clip(-z[2], -1, 1)))

    def settle(self, name, cond, q_hold, timeout, hold):
        """Hold q_hold under control; success only when cond() holds continuously for `hold`."""
        t0, held_since = self.d.time, None
        z = np.zeros(7)
        while True:
            self.step(q_hold, z, z)
            ok, info = cond()
            if ok:
                held_since = self.d.time if held_since is None else held_since
                if self.d.time - held_since >= hold - 1e-9:
                    return info
            else:
                held_since = None
            if self.d.time - t0 > timeout:
                raise Failed(f"{name}_timeout", **info)

    def home_cond(self):
        q, qd = self.rob.q(self.d), self.rob.qd(self.d)
        eq, eqd = float(np.max(np.abs(q - self.rob.home))), float(np.max(np.abs(qd)))
        return (eq <= C.Q_TOL and eqd <= C.QD_TOL), {"max_q_err": eq, "max_qd": eqd}

    def reach_cond(self):
        p, eo = self.errors()
        ep = float(np.linalg.norm(p - self.target))
        eqd = float(np.max(np.abs(self.rob.qd(self.d))))
        q = self.rob.q(self.d)
        inlim = bool(np.all(q >= self.rob.lo) and np.all(q <= self.rob.hi))
        ok = ep <= C.POS_TOL and eo <= C.ORI_TOL and eqd <= C.QD_TOL and inlim
        return ok, {"pos_err": ep, "ori_err_deg": math.degrees(eo), "max_qd": eqd}

    def move(self, q_goal, label):
        """Plan from the *measured* state, re-check path, then track it with torque control."""
        q0, qd0 = self.rob.q(self.d), self.rob.qd(self.d)
        traj = Quintic(q0, qd0, q_goal, duration(self.rob, q0, q_goal))
        ok, why, info = check_path(self.rob, traj)
        if not ok:
            raise Failed(f"{label}_path_invalid", reason=why, **info)
        self.ev(f"{label}_TRAJ", T=round(traj.T, 3), start_qd_max=float(np.max(np.abs(qd0))), **info)
        t0 = self.d.time
        self.sat_steps = 0
        while self.d.time - t0 < traj.T:
            self.step(*traj.at(self.d.time - t0))
        return traj

    def run(self):
        a = self.args
        seed = a.seed if a.seed is not None else int(time.time_ns() % (2 ** 32))
        rng = np.random.default_rng(seed)  # created once
        # initialisation: start pose set ONCE (not a recovery)
        self.d.qpos[self.rob.qadr] = self.rob.home
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)
        self.warn0 = int(sum(w.number for w in self.d.warning))
        self.ev("START", mode=a.mode, seed=seed, mujoco=mujoco.__version__, init="qpos=home,qvel=0 (start pose only)")
        if not a.headless:
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
            with self.viewer.lock():
                self.viewer.cam.lookat[:] = [0.3, 0.0, 0.35]
                self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 1.8, 140, -20
        self.shadow = (self.d.time, self.d.qpos.copy(), self.d.qvel.copy())
        self.sim_wall0, self.last_sync = time.monotonic(), 0.0

        self.phase = "HOME_CHECK"
        self.status("HOME_CHECK")
        info = self.settle("home_check", self.home_cond, self.rob.home, C.HOME_TIMEOUT, C.HOME_HOLD)
        self.ev("HOME_CONFIRMED", **info)
        if a.mode == "view":
            self.phase = "HOLD"
            z = np.zeros(7)
            while True:
                self.step(self.rob.home, z, z)

        while True:  # one iteration per cycle; reached only after home is confirmed
            self.cycle += 1
            self.phase = "GEN_TARGET"
            self.status(f"cycle {self.cycle}: GEN_TARGET")
            q0, qd0 = self.rob.q(self.d), self.rob.qd(self.d)
            if a.mode == "fixed":
                from .core import validate_goal
                p = np.array(C.FIXED_TARGET)
                q_goal, why, ginfo = validate_goal(self.rob, p, q0, qd0, self.stop)
                if q_goal is None:
                    raise Failed("fixed_target_invalid", reason=why, **ginfo)
            else:
                p, q_goal, ginfo = generate_target(self.rob, rng, q0, qd0, self.stop,
                                                   lambda n, **k: self.ev(n, **k))
                if p is None:
                    if q_goal == "stopped":
                        raise KeyboardInterrupt
                    raise Failed(q_goal, rejects=ginfo)
            self.target, self.q_goal = p, q_goal
            self.d.mocap_pos[0] = p  # marker only (not robot state)
            self.resync_clock()
            self.ev("TARGET_GENERATED", target=p, q_goal=q_goal, **ginfo)

            self.phase = "REACH"
            self.status(f"cycle {self.cycle}: REACH")
            traj = self.move(q_goal, "REACH")
            self.phase = "REACH_SETTLE"
            info = self.settle("reach", self.reach_cond, q_goal, C.SETTLE_EXTRA, C.HOLD)
            self.ev("REACH_CONFIRMED", sat_steps=self.sat_steps, T=round(traj.T, 3), **info)

            self.phase = "RECOVER"
            self.status(f"cycle {self.cycle}: RECOVER")
            self.ev("RECOVERY_STARTED", q=self.rob.q(self.d), qd=self.rob.qd(self.d))
            traj = self.move(self.rob.home, "RECOVER")
            self.phase = "RECOVER_SETTLE"
            info = self.settle("recover", self.home_cond, self.rob.home, C.SETTLE_EXTRA, C.HOLD)
            self.ev("RECOVERY_CONFIRMED", sat_steps=self.sat_steps, T=round(traj.T, 3), **info)
            if a.mode == "fixed":
                self.stop.request("fixed_mode_done")
                raise KeyboardInterrupt

    def fail(self, e):
        p, eo = self.errors()
        rec = {"phase": self.phase, "cycle": self.cycle, "confirmed_cause": e.cause, "detail": e.info,
               "target": None if self.target is None else np.round(self.target, 5).tolist(),
               "q": self.rob.q(self.d).round(5).tolist(), "qd": self.rob.qd(self.d).round(5).tolist(),
               "tcp": p.round(5).tolist(), "tcp_ori_err_deg": round(math.degrees(eo), 3),
               "q_err_home": float(np.max(np.abs(self.rob.q(self.d) - self.rob.home))),
               "sim_t": self.d.time, "wall_t": round(time.monotonic() - self.wall_t0, 3),
               "suspected_cause": {"tracking_error": "gain/trajectory too aggressive or external disturbance",
                                   "unexpected_contact": "path sampling missed a collision or controller overshoot",
                                   "target_generation_exhausted": "target box mostly unreachable under constraints",
                                   }.get(e.cause, "see detail")}
        self.ev("FAILED", **rec)
        (C.LOG_DIR / f"failure_{self.stamp}.json").write_text(json.dumps(rec, indent=2, default=str))
        self.phase = "FAILED"
        self.status(f"FAILED: {e.cause} (physics stopped; close window or Ctrl+C)")
        if self.viewer is not None and not self.args.exit_on_fail:
            while self.viewer.is_running() and not self.stop():
                self.viewer.sync()  # no physics stepping
                time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["random", "fixed", "view"], default="random")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--exit-on-fail", action="store_true")
    a = ap.parse_args()
    sim = Sim(a)
    signal.signal(signal.SIGINT, lambda *_: sim.stop.request("SIGINT"))
    signal.signal(signal.SIGTERM, lambda *_: sim.stop.request("SIGTERM"))
    code = 0
    try:
        sim.run()
    except KeyboardInterrupt:
        if not sim.stop():
            sim.stop.request("SIGINT")
    except Failed as e:
        code = 1
        sim.fail(e)
    except Exception:
        code = 1
        sim.log_.exception("UNHANDLED_EXCEPTION")
    finally:
        t = time.monotonic()
        if sim.viewer is not None:
            sim.viewer.close()
            while sim.viewer.is_running() and time.monotonic() - t < 3.0:  # wait for render thread to exit
                time.sleep(0.01)
        sim.ev("SHUTDOWN", reason=sim.stop.reason or ("failed" if code else "unknown"), exit_code=code, cycles=sim.cycle,
               close_s=round(time.monotonic() - t, 3))
        sim.ev("VIEWER_CLOSED", running=bool(sim.viewer is not None and sim.viewer.is_running()))
        logging.shutdown()
        sys.stdout.flush()
    # Resources are released above. os._exit skips interpreter finalization, where GLFW/GL
    # teardown segfaulted (exit 139) with mujoco 3.15.0 passive viewer on this PC.
    os._exit(code)


if __name__ == "__main__":
    main()
