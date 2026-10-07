"""Convert expanded official URDF (fr3_hand.urdf) to a MuJoCo scene.

Outputs (all inside models/):
  meshes/visual/<link>_<k>.obj   per-material parts of each official DAE (scene transforms baked in)
  meshes/collision/<file>.stl    official collision STLs (copied)
  fr3_hand_scene.xml             robot + floor + light + target marker, meshdir="meshes"
Every modification relative to the official URDF is listed in docs/MODEL_CONVERSION.md.
"""
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import yaml

WS = Path(__file__).resolve().parents[1]
FD = WS / "third_party/franka_description"
MODELS = WS / "models"
MESHDIR = MODELS / "meshes"
FINGER_OPEN = 0.04  # m, each finger (fixed fully-open)
TCP_OFFSET = 0.1034  # m, from expanded URDF fr3_hand_tcp_joint (checked below)


def pkg_path(uri):
    return FD / uri.replace("package://franka_description/", "")


def rpy_quat(rpy):
    r, p, y = rpy
    def q(axis, a):
        v = np.zeros(4); v[0] = math.cos(a / 2); v[1 + axis] = math.sin(a / 2); return v
    out = np.zeros(4)
    tmp = np.zeros(4)
    mujoco.mju_mulQuat(tmp, q(2, y), q(1, p))
    mujoco.mju_mulQuat(out, tmp, q(0, r))
    return out


def convert_dae(dae, stem):
    """Split DAE into one OBJ per geometry node; returns [(relpath, rgba)]."""
    scene = trimesh.load(dae, force="scene")
    parts = []
    for k, node in enumerate(scene.graph.nodes_geometry):
        T, gname = scene.graph[node]
        mesh = scene.geometry[gname].copy()
        mesh.apply_transform(T)
        mat = getattr(mesh.visual, "material", None)
        rgba = (np.array(mat.main_color, float) / 255.0) if mat is not None else np.array([.8, .8, .8, 1])
        rel = f"visual/{stem}_{k}.obj"
        out = MESHDIR / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        trimesh.Trimesh(mesh.vertices, mesh.faces, process=False).export(out)
        parts.append((rel, rgba))
    return parts


def main():
    tree = ET.parse(MODELS / "urdf/fr3_hand.urdf")
    root = tree.getroot()
    # TCP offset check against expanded URDF
    tcpj = [j for j in root.findall("joint") if j.get("name") == "fr3_hand_tcp_joint"][0]
    assert abs(float(tcpj.find("origin").get("xyz").split()[2]) - TCP_OFFSET) < 1e-9

    # 1) drop massless helper links (base, accelerometers, hand_tcp -> becomes a site)
    drop = {l.get("name") for l in root.findall("link")
            if l.get("name") == "base" or "accelerometer" in l.get("name") or l.get("name") == "fr3_hand_tcp"}
    for l in [l for l in root.findall("link") if l.get("name") in drop]:
        root.remove(l)
    for j in [j for j in root.findall("joint") if j.find("child").get("link") in drop or j.find("parent").get("link") in drop]:
        root.remove(j)

    # 2) fingers: prismatic -> fixed at FINGER_OPEN (transform of official joint at q=0.04)
    for j in root.findall("joint"):
        if j.get("name") in ("fr3_finger_joint1", "fr3_finger_joint2"):
            o = j.find("origin")
            xyz = [float(v) for v in o.get("xyz").split()]
            yaw = float(o.get("rpy").split()[2])
            # axis y in child frame, child rotated by yaw about z
            xyz[0] += -math.sin(yaw) * FINGER_OPEN
            xyz[1] += math.cos(yaw) * FINGER_OPEN
            o.set("xyz", " ".join(f"{v:.6g}" for v in xyz))
            j.set("type", "fixed")
            for tag in ("axis", "limit", "mimic", "dynamics"):
                e = j.find(tag)
                if e is not None:
                    j.remove(e)

    # 3) visuals -> converted OBJ parts (added later via MjSpec); collision STL -> copied
    visuals = {}
    for l in root.findall("link"):
        for v in l.findall("visual"):
            uri = v.find("geometry/mesh").get("filename")
            o = v.find("origin")
            pos = [float(x) for x in o.get("xyz").split()] if o is not None else [0, 0, 0]
            rpy = [float(x) for x in o.get("rpy").split()] if o is not None else [0, 0, 0]
            stem = l.get("name")
            visuals[l.get("name")] = (convert_dae(pkg_path(uri), stem), pos, rpy)
            l.remove(v)
        for c in l.findall("collision"):
            m = c.find("geometry/mesh")
            if m is not None:
                src = pkg_path(m.get("filename"))
                rel = f"collision/{l.get('name')}.stl"
                (MESHDIR / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(src, MESHDIR / rel)
                m.set("filename", rel)

    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", meshdir=str(MESHDIR), discardvisual="false",
                  fusestatic="false", balanceinertia="false", strippath="false")
    (WS / ".tmp").mkdir(exist_ok=True)
    tmp_urdf = WS / ".tmp/fr3_hand_pre.urdf"
    tree.write(tmp_urdf)

    spec = mujoco.MjSpec.from_file(str(tmp_urdf))
    spec.modelname = "fr3_hand"
    spec.meshdir = str(MESHDIR)  # absolute while compiling; relative "meshes" in saved XML
    spec.compiler.autolimits = True
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for m in spec.meshes:  # make collision mesh paths relative to meshdir
        m.file = str(Path(m.file).relative_to(MESHDIR)) if Path(m.file).is_absolute() else m.file

    # collision geoms: group 3, collide with everything
    for g in spec.geoms:
        g.group = 3
        g.contype = 1
        g.conaffinity = 1
        g.rgba = [0.5, 0.5, 0.9, 0.4]

    # visual geoms: group 2, no collision, no mass contribution
    for link, (parts, pos, rpy) in visuals.items():
        body = spec.body(link)
        quat = rpy_quat(rpy)
        for rel, rgba in parts:
            name = Path(rel).stem
            spec.add_mesh(name=name, file=rel)
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, pos=pos, quat=quat,
                          contype=0, conaffinity=0, group=2, density=0, mass=0, rgba=rgba)

    # TCP site
    spec.body("fr3_hand").add_site(name="tcp", pos=[0, 0, TCP_OFFSET], size=[0.006, 0, 0],
                                   rgba=[1, 0.6, 0, 1], group=1)

    # joints: armature (motor_inertia * gear^2 from dynamics.yaml, derived), damping (proposal)
    dyn = yaml.safe_load(open(FD / "robots/fr3/dynamics.yaml"))
    lim = yaml.safe_load(open(FD / "robots/fr3/joint_limits.yaml"))
    for i in range(1, 8):
        d = dyn[f"joint{i}"]["dynamic"]
        j = spec.joint(f"fr3_joint{i}")
        j.armature = d["motor_inertia"] * d["gear_ratio"] ** 2
        j.damping = np.array([0.1, 0.0, 0.0])
        eff = lim[f"joint{i}"]["limit"]["effort"]
        spec.add_actuator(name=f"act{i}", target=f"fr3_joint{i}",
                          trntype=mujoco.mjtTrn.mjTRN_JOINT, gear=[1, 0, 0, 0, 0, 0],
                          ctrllimited=True, ctrlrange=[-eff, eff])

    # SRDF disable_collisions -> exclude pairs (skip bodies without geoms, e.g. link8)
    srdf = ET.parse(MODELS / "urdf/fr3_hand.srdf").getroot()
    bodies = {b.name for b in spec.bodies}
    n_ex = 0
    for dc in srdf.findall("disable_collisions"):
        a, b = dc.get("link1"), dc.get("link2")
        if a in bodies and b in bodies:
            spec.add_exclude(bodyname1=a, bodyname2=b)
            n_ex += 1

    # scene: floor at base height (link0 origin on z=0), light, mocap target marker
    wb = spec.worldbody
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[.25, .27, .3], rgb2=[.32, .34, .38], width=300, height=300)
    mat = spec.add_material(name="grid")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [8, 8]
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.05], material="grid",
                contype=1, conaffinity=1, group=0)
    spec.add_exclude(bodyname1="world", bodyname2="fr3_link0")  # allowed: base mounted on floor
    wb.add_light(pos=[0, 0, 2.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, diffuse=[.8, .8, .8])
    tgt = wb.add_body(name="target", mocap=True, pos=[0.45, 0.2, 0.35])
    tgt.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.012, 0, 0], rgba=[1, 0.1, 0.1, 0.6],
                 contype=0, conaffinity=0, group=1, density=0)
    tgt.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.003, 0.04, 0], pos=[0, 0, 0.04],
                 rgba=[1, 0.1, 0.1, 0.6], contype=0, conaffinity=0, group=1, density=0)
    spec.visual.global_.offwidth = 1280

    spec.compile()
    xml = spec.to_xml().replace(f'meshdir="{MESHDIR}/"', 'meshdir="meshes/"').replace(f'meshdir="{MESHDIR}"', 'meshdir="meshes"')
    assert str(MESHDIR) not in xml
    (MODELS / "fr3_hand_scene.xml").write_text(xml)
    m = mujoco.MjModel.from_xml_path(str(MODELS / "fr3_hand_scene.xml"))
    print(f"ok: nq={m.nq} nu={m.nu} nbody={m.nbody} ngeom={m.ngeom} nmesh={m.nmesh} excludes={n_ex}")


if __name__ == "__main__":
    main()
