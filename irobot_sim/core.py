"""Model, controller, trajectory, IK and validity checks.

Live MjData is only written at initialisation (start pose) and through ctrl.
IK and path checks use a separate scratch model/data (with contact margin CLEARANCE).
"""
import math
import time

import mujoco
import numpy as np

from . import config as C


class Robot:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(C.SCENE))
        self.qadr = np.array([self.m.jnt_qposadr[self.m.joint(j).id] for j in C.JOINTS])
        self.dadr = np.array([self.m.jnt_dofadr[self.m.joint(j).id] for j in C.JOINTS])
        self.lo = self.m.jnt_range[[self.m.joint(j).id for j in C.JOINTS], 0].copy()
        self.hi = self.m.jnt_range[[self.m.joint(j).id for j in C.JOINTS], 1].copy()
        self.tau_max = self.m.actuator_ctrlrange[:, 1].copy()
        self.site = self.m.site("tcp").id
        self.home = np.array(C.HOME)
        self.vmax = np.array(C.VMAX)
        # scratch model: same model, contact margin -> clearance check
        self.ms = mujoco.MjModel.from_xml_path(str(C.SCENE))
        self.ms.geom_margin[:] = C.CLEARANCE
        self.ds = mujoco.MjData(self.ms)
        self.floor = self.m.geom("floor").id
        near = {"fr3_link5", "fr3_link6", "fr3_link7", "fr3_hand", "fr3_leftfinger", "fr3_rightfinger"}
        self.floor_check_geoms = [g for g in range(self.m.ngeom)
                                  if self.m.geom_contype[g] and self.m.body(self.m.geom_bodyid[g]).name in near]
        self._M = np.zeros((self.m.nv, self.m.nv))

    # ---------- live state helpers ----------
    def q(self, d):
        return d.qpos[self.qadr].copy()

    def qd(self, d):
        return d.qvel[self.dadr].copy()

    def tcp(self, d):
        return d.site_xpos[self.site].copy(), d.site_xmat[self.site].reshape(3, 3)[:, 2].copy()

    def control(self, d, qd_des, qdd_des, q_des):
        """Computed-torque PD + bias. Returns (tau, saturated)."""
        mujoco.mj_fullM(self.m, d, self._M)
        M = self._M[np.ix_(self.dadr, self.dadr)]
        e = q_des - self.q(d)
        ed = qd_des - self.qd(d)
        tau = M @ (qdd_des + C.KP * e + C.KD * ed) + d.qfrc_bias[self.dadr]
        sat = bool(np.any(np.abs(tau) > self.tau_max))
        tau = np.clip(tau, -self.tau_max, self.tau_max)
        d.ctrl[:] = tau
        return tau, sat

    # ---------- scratch kinematics ----------
    def fk_scratch(self, q):
        self.ds.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.ms, self.ds)
        mujoco.mj_comPos(self.ms, self.ds)
        return self.ds.site_xpos[self.site].copy(), self.ds.site_xmat[self.site].reshape(3, 3)[:, 2].copy()

    def contacts_scratch(self, q):
        """Contacts (incl. CLEARANCE margin) at q. Returns list of (body1, body2, dist)."""
        self.ds.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.ms, self.ds)
        mujoco.mj_collision(self.ms, self.ds)
        out = []
        for c in self.ds.contact[: self.ds.ncon]:
            b1 = self.ms.body(self.ms.geom_bodyid[c.geom1]).name
            b2 = self.ms.body(self.ms.geom_bodyid[c.geom2]).name
            out.append((b1, b2, float(c.dist)))
        return out

    def floor_distance_scratch(self, q):
        self.ds.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.ms, self.ds)
        fromto = np.zeros(6)
        return min(mujoco.mj_geomDistance(self.ms, self.ds, self.floor, g, 0.5, fromto)
                   for g in self.floor_check_geoms)

    def ik(self, p_des, q0, stop=lambda: False, iters=200):
        zd = np.array([0.0, 0.0, -1.0])
        q = q0.copy()
        jp = np.zeros((3, self.ms.nv)); jr = np.zeros((3, self.ms.nv))
        for _ in range(iters):
            if stop():
                return None, np.inf, np.inf
            p, z = self.fk_scratch(q)
            ep = p_des - p
            eo = np.cross(z, zd)
            if np.linalg.norm(ep) < 2e-4 and np.linalg.norm(eo) < 2e-3:
                break
            mujoco.mj_jacSite(self.ms, self.ds, jp, jr, self.site)
            P = np.eye(3) - np.outer(z, z)
            J = np.vstack([jp[:, self.dadr], P @ jr[:, self.dadr]])
            e = np.concatenate([ep, eo])
            JJt = J @ J.T + (0.05 ** 2) * np.eye(6)
            Jp = J.T @ np.linalg.inv(JJt)
            dq = Jp @ e + (np.eye(7) - Jp @ J) @ (0.1 * (self.home - q))
            n = np.linalg.norm(dq)
            if n > 0.2:
                dq *= 0.2 / n
            q = np.clip(q + dq, self.lo + 1e-3, self.hi - 1e-3)
        p, z = self.fk_scratch(q)
        return q, float(np.linalg.norm(p_des - p)), float(math.acos(np.clip(-z[2], -1, 1)))


class Quintic:
    """Joint quintic from (q0, v0, a0=0) to (q1, 0, 0) over T."""

    def __init__(self, q0, v0, q1, T):
        self.q0, self.T = q0, T
        dq = q1 - q0
        self.c = (q0, v0, np.zeros_like(q0),
                  (20 * dq - 12 * v0 * T) / (2 * T ** 3),
                  (-30 * dq + 16 * v0 * T) / (2 * T ** 4),
                  (12 * dq - 6 * v0 * T) / (2 * T ** 5))
        self.q1 = q1

    def at(self, t):
        if t >= self.T:
            z = np.zeros_like(self.q0)
            return self.q1.copy(), z, z
        a0, a1, a2, a3, a4, a5 = self.c
        q = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
        v = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
        a = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
        return q, v, a


def duration(rob, q0, q1):
    return max(C.T_MIN, 1.875 * float(np.max(np.abs(q1 - q0) / (C.SPEED_FRAC * rob.vmax))))


def check_path(rob, traj):
    """Sample traj densely (<= PATH_DQ per joint between samples, >= 50 samples).
    Finite sampling: not a proof of continuous-path safety. Returns (ok, reason, info)."""
    span = float(np.max(np.abs(traj.q1 - traj.q0)))
    n = max(50, int(math.ceil(span / C.PATH_DQ)) * 2)
    vmax_seen = 0.0
    for k in range(n + 1):
        q, v, _ = traj.at(traj.T * k / n)
        vmax_seen = max(vmax_seen, float(np.max(np.abs(v) / rob.vmax)))
        if np.any(q < rob.lo + 1e-3) or np.any(q > rob.hi - 1e-3):
            return False, "path_limit", {"k": k, "n": n}
        c = rob.contacts_scratch(q)
        if c:
            return False, "path_collision", {"k": k, "n": n, "pair": c[0]}
    if vmax_seen > 0.95:
        return False, "path_velocity", {"vfrac": vmax_seen}
    return True, "", {"n": n, "vfrac": round(vmax_seen, 3)}


def validate_goal(rob, p_des, q_start, qd_start, stop):
    """IK + checks for target p_des from the *measured* start state."""
    starts = [rob.home.copy(), q_start.copy()]
    best = None
    for q0 in starts:
        q, ep, eo = rob.ik(p_des, q0, stop)
        if q is None:
            return None, "stopped", {}
        if best is None or ep + eo * 0.1 < best[1] + best[2] * 0.1:
            best = (q, ep, eo)
        if ep < 1e-3 and eo < math.radians(1):
            break
    q, ep, eo = best
    info = {"ik_pos_err": ep, "ik_ori_err_deg": math.degrees(eo)}
    if ep >= 1e-3 or eo >= math.radians(1):
        return None, "ik_not_converged", info
    if np.any(q < rob.lo + C.LIMIT_MARGIN) or np.any(q > rob.hi - C.LIMIT_MARGIN):
        return None, "goal_limit_margin", info
    c = rob.contacts_scratch(q)
    if c:
        info["pair"] = c[0]
        return None, "goal_collision", info
    fd = rob.floor_distance_scratch(q)
    info["floor_dist"] = fd
    if fd < C.FLOOR_CLEAR:
        return None, "goal_floor_clearance", info
    traj = Quintic(q_start, qd_start, q, duration(rob, q_start, q))
    ok, why, pinfo = check_path(rob, traj)
    info.update(pinfo)
    if not ok:
        return None, why, info
    info["T"] = traj.T
    return q, "", info


def sample_target(rng):
    while True:  # rejection on the radius only (cheap, geometric)
        p = np.array([rng.uniform(*C.BOX["x"]), rng.uniform(*C.BOX["y"]), rng.uniform(*C.BOX["z"])])
        if math.hypot(p[0], p[1]) >= C.R_MIN:
            return p


def generate_target(rob, rng, q_start, qd_start, stop, log):
    """Bounded search. Returns (p, q_goal, info) or (None, reason, stats)."""
    t0 = time.monotonic()
    rejects = {}
    for k in range(1, C.MAX_CANDIDATES + 1):
        if stop():
            return None, "stopped", rejects
        if time.monotonic() - t0 > C.GEN_WALL_LIMIT:
            return None, "target_generation_time_limit", rejects
        p = sample_target(rng)
        q, why, info = validate_goal(rob, p, q_start, qd_start, stop)
        if q is not None:
            info.update(candidates=k, gen_wall=round(time.monotonic() - t0, 3), rejects=rejects)
            return p, q, info
        if why == "stopped":
            return None, "stopped", rejects
        rejects[why] = rejects.get(why, 0) + 1
        log("CANDIDATE_REJECTED", k=k, target=p.round(4).tolist(), reason=why)
    return None, "target_generation_exhausted", rejects
