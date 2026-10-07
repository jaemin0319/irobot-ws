"""Model checks -> logs/model_check.json (exit 1 if any check fails)."""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))
from irobot_sim import config as C  # noqa: E402
from irobot_sim.core import Robot  # noqa: E402

FD = WS / "third_party/franka_description"
res, ok_all = {}, True


def chk(name, ok, **info):
    global ok_all
    ok_all &= bool(ok)
    res[name] = {"ok": bool(ok), **info}
    print(("PASS " if ok else "FAIL ") + name, json.dumps(info, default=str)[:300])


rob = Robot()
m, d = rob.m, mujoco.MjData(rob.m)
urdf = ET.parse(WS / "models/urdf/fr3_hand.urdf").getroot()

# structure
chk("structure", m.nq == 7 and m.nv == 7 and m.nu == 7, nq=m.nq, nv=m.nv, nu=m.nu, nmesh=m.nmesh, ngeom=m.ngeom)
vis = [g for g in range(m.ngeom) if m.geom_group[g] == 2]
chk("visual_geoms_no_collision", all(m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0 for g in vis), n=len(vis))
col = [g for g in range(m.ngeom) if m.geom_group[g] == 3]
chk("collision_geoms_active", len(col) > 0 and all(m.geom_contype[g] and m.geom_conaffinity[g] for g in col), n=len(col))

# joint limits
lim = yaml.safe_load(open(FD / "robots/fr3/joint_limits.yaml"))
dl = [max(abs(rob.lo[i] - lim[f"joint{i+1}"]["limit"]["lower"]), abs(rob.hi[i] - lim[f"joint{i+1}"]["limit"]["upper"])) for i in range(7)]
chk("joint_limits", max(dl) < 1e-4 and all(m.jnt_limited[m.joint(j).id] for j in C.JOINTS), max_diff=max(dl))

# actuators
eff = [lim[f"joint{i}"]["limit"]["effort"] for i in range(1, 8)]
amap = [m.joint(m.actuator_trnid[a, 0]).name for a in range(m.nu)]
chk("actuators", amap == C.JOINTS and np.allclose(m.actuator_ctrlrange[:, 1], eff) and np.allclose(m.actuator_gear[:, 0], 1),
    map=amap, ctrlrange=m.actuator_ctrlrange[:, 1].tolist())
chk("armature_derived", True, armature=[round(float(m.dof_armature[i]), 4) for i in rob.dadr],
    note="motor_inertia[kg m^2, motor side] * gear_ratio^2 from dynamics.yaml (derived, not an official joint value)")

# inertials: compare mass, COM, full inertia tensor in link frame
def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])

worst = {"mass": 0, "com": 0, "inertia": 0}
for link in urdf.findall("link"):
    ine = link.find("inertial")
    name = link.get("name")
    if ine is None or mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
        continue
    b = m.body(name)
    mass = float(ine.find("mass").get("value"))
    com = np.array([float(v) for v in ine.find("origin").get("xyz").split()])
    I = ine.find("inertia").attrib
    Iu = np.array([[float(I["ixx"]), float(I["ixy"]), float(I["ixz"])], [float(I["ixy"]), float(I["iyy"]), float(I["iyz"])], [float(I["ixz"]), float(I["iyz"]), float(I["izz"])]])
    R = np.zeros(9); mujoco.mju_quat2Mat(R, b.iquat); R = R.reshape(3, 3)
    Im = R @ np.diag(b.inertia) @ R.T
    worst["mass"] = max(worst["mass"], abs(b.mass[0] - mass) / mass)
    worst["com"] = max(worst["com"], float(np.linalg.norm(b.ipos - com)))
    worst["inertia"] = max(worst["inertia"], float(np.max(np.abs(Im - Iu)) / np.max(np.abs(Iu))))
chk("inertials_match_urdf", worst["mass"] < 0.01 and worst["com"] < 1e-6 and worst["inertia"] < 1e-3, **worst)

# finger transforms (fixed at 0.04 open)
d.qpos[rob.qadr] = rob.home
mujoco.mj_forward(m, d)
lf, rf = m.body("fr3_leftfinger").pos, m.body("fr3_rightfinger").pos
chk("finger_fixed_open", np.allclose(lf, [0, 0.04, 0.0584], atol=1e-6) and np.allclose(rf, [0, -0.04, 0.0584], atol=1e-6),
    left=lf.tolist(), right=rf.tolist())

# independent FK from URDF joint chain (numpy) vs MuJoCo tcp site
joints = {j.get("child"): j for j in []}
chain = ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7",
         "fr3_joint8", "fr3_hand_joint", "fr3_hand_tcp_joint"]
J = {j.get("name"): j for j in urdf.findall("joint")}
T = np.eye(4)
for i, n in enumerate(chain):
    o = J[n].find("origin")
    xyz = [float(v) for v in o.get("xyz").split()]
    rpy = [float(v) for v in o.get("rpy").split()]
    A = np.eye(4); A[:3, :3] = rpy_R(*rpy); A[:3, 3] = xyz
    T = T @ A
    if i < 7:
        c, s = math.cos(rob.home[i]), math.sin(rob.home[i])
        Rz = np.eye(4); Rz[:2, :2] = [[c, -s], [s, c]]
        T = T @ Rz
p_mj, z_mj = rob.tcp(d)
chk("tcp_fk_independent", np.linalg.norm(T[:3, 3] - p_mj) < 1e-4 and np.linalg.norm(T[:3, 2] - z_mj) < 1e-4,
    urdf_fk=T[:3, 3].round(5).tolist(), mujoco=p_mj.round(5).tolist(), tool_z=z_mj.round(4).tolist())

# contacts at home (live margin 0, scratch margin CLEARANCE); world-link0 is the only allowed mount contact
chk("home_no_contact", d.ncon == 0, ncon=d.ncon)
cs = rob.contacts_scratch(rob.home)
chk("home_clearance", not cs, contacts=cs)
base_z = d.xpos[m.body("fr3_link0").id][2]
chk("base_on_floor", abs(base_z) < 1e-9, link0_z=base_z, floor_z=0.0,
    note="link0 mesh bottom touches floor plane; world<->fr3_link0 excluded as the mount")

# collision detection works: fold joint2/4 to drive the hand into link0/floor, and a self-collision pose
pose_floor = rob.home.copy(); pose_floor[1] = 1.7; pose_floor[3] = -0.2
c1 = rob.contacts_scratch(pose_floor)
pose_self = rob.home.copy(); pose_self[1] = 0.0; pose_self[3] = -3.07; pose_self[5] = 0.44  # link1<->link6/7 (not excluded)
c2 = [c for c in rob.contacts_scratch(pose_self) if c[2] < 0]
chk("collision_detection_works", bool(c1) and bool(c2), floor_pose=c1[:2], self_pose=c2[:2])

# controllability: hold home 10 s under PD+bias from rest
d2 = mujoco.MjData(m); d2.qpos[rob.qadr] = rob.home; mujoco.mj_forward(m, d2)
z = np.zeros(7); sat = 0; worst_e = worst_v = 0.0
while d2.time < 10.0:
    _, s = rob.control(d2, z, z, rob.home); sat += s
    mujoco.mj_step(m, d2)
    worst_e = max(worst_e, float(np.max(np.abs(rob.q(d2) - rob.home))))
    worst_v = max(worst_v, float(np.max(np.abs(rob.qd(d2)))))
chk("hold_home_10s", worst_e < 0.005 and worst_v < 0.01 and sat == 0 and np.all(np.isfinite(d2.qpos)),
    max_q_err=worst_e, max_qd=worst_v, sat_steps=sat)

(WS / "logs").mkdir(exist_ok=True)
(WS / "logs/model_check.json").write_text(json.dumps({"all_ok": ok_all, "checks": res}, indent=2, default=str))
print("ALL OK" if ok_all else "SOME CHECKS FAILED")
sys.exit(0 if ok_all else 1)
