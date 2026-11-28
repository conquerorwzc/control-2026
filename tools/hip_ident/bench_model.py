"""Prepare and check the fixed-base identification derivative of the 1127 CAD model."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np


def set_armature(model, scratch, dofs, armature):
    """Refresh mass-dependent constraint weights whenever reflected inertia changes."""
    model.dof_armature[dofs] = armature
    mj.mj_setConst(model, scratch)


def prepare(folder, output, installed=False):
    source = folder / "wheel_leg_1127_battery.xml"
    urdf = folder / "wheel_leg_1127_battery.urdf"
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("worldbody")
    base = world.find("body[@name='base_link']")
    full = mj.MjModel.from_xml_path(str(source))
    links = ET.parse(urdf).getroot().findall("link")
    urdf_mass = sum(float(x.get("value")) for link in links for x in link.findall("inertial/mass"))
    if abs(urdf_mass - full.body_mass.sum()) > 0.0001:
        raise ValueError("URDF/MJCF mass mismatch")
    base.remove(base.find("freejoint"))
    world.remove(world.find("geom[@name='ground']"))
    base.set("pos", "0 0 0")
    eq = root.find("equality")
    removed_mass = 0.0
    for side in ("left", "right"):
        hip = base.find(f"body[@name='{side}_hip']")
        rod = hip.find(f"body[@name='{side}_spring_rod']")
        if installed:
            # The exported slide damper is a modeling guess. Apply identified axial forces separately.
            slide = rod.find(".//joint[@type='slide']")
            slide.set("damping", "0")
        else:
            removed_mass += sum(float(x.get("mass")) for x in rod.iter("inertial"))
            hip.remove(rod)
            eq.remove(eq.find(f"connect[@name='{side}_spring_end_loop']"))
    root.remove(root.find("actuator"))
    # Fixed rig has no contacts. Every remaining physical body has an explicit CAD inertia.
    for body in root.iter("body"):
        if body.find("inertial") is None:
            raise ValueError("missing CAD inertia on " + body.get("name"))
        for geom in body.findall("geom"):
            body.remove(geom)
    root.remove(root.find("asset"))
    root.find("compiler").attrib.pop("meshdir", None)
    root.find("compiler").set("fusestatic", "false")
    root.find("option").set("jacobian", "dense")
    root.set("model", "wheel_leg_1127_battery_fixed_" + ("installed" if installed else "removed"))
    xml = ET.tostring(root, encoding="unicode")
    model = mj.MjModel.from_xml_string(xml)
    if (
        model.neq != (10 if installed else 8)
        or model.nu
        or abs(model.body_mass.sum() + removed_mass - full.body_mass.sum()) > 1e-7
    ):
        raise ValueError("fixed-rig topology or mass mismatch")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml, encoding="utf-8")
    return dict(
        urdf_links=len(links),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        cad_kg=urdf_mass,
        removed_kg=removed_mass,
        bench_kg=float(model.body_mass.sum()),
        constraints=model.neq,
        timestep_s=model.opt.timestep,
    )


def closure(model, data):
    mj.mj_forward(model, data)
    eq = data.efc_type == int(mj.mjtConstraint.mjCNSTR_EQUALITY)
    return data.efc_pos[eq], data.efc_J.reshape(-1, model.nv)[eq]


def set_pose(model, data, joints, target, velocity=None):
    """Continue from this assembly branch. Hold measured shafts fixed, project the passive joints."""
    ids = np.array([model.joint(j + "_sprocket_joint").id for j in joints])
    qi, vi = model.jnt_qposadr[ids], model.jnt_dofadr[ids]
    movable = np.setdiff1d(np.arange(model.nv), vi)
    if model.nq != model.nv:
        raise ValueError("expected a fixed rig of 1-DOF joints")
    start = data.qpos[qi].copy()
    for fraction in np.linspace(0, 1, max(2, 1 + int(np.ceil(np.max(np.abs(target - start)) / 0.08))))[1:]:
        data.qpos[qi] = start + fraction * (target - start)
        for _ in range(35):
            residual, jac = closure(model, data)
            if np.max(np.abs(residual)) < 1e-8:
                break
            step = np.linalg.lstsq(jac[:, movable], -residual, rcond=1e-8)[0]
            data.qpos[movable] += step / max(1.0, np.max(np.abs(step)) / 0.15)
        else:
            raise ValueError("could not close linkage at measured initial pose")
        limited = np.flatnonzero(model.jnt_limited)
        pos = data.qpos[model.jnt_qposadr[limited]]
        if np.any(pos < model.jnt_range[limited, 0] - 0.0001) or np.any(
            pos > model.jnt_range[limited, 1] + 0.0001
        ):
            raise ValueError("mapped pose outside CAD knee travel; check alignment")
    tangent = np.zeros((model.nv, len(ids)))
    tangent[vi] = np.eye(len(ids))
    tangent[movable] = np.linalg.lstsq(jac[:, movable], -jac[:, vi], rcond=1e-8)[0]
    if velocity is not None:
        data.qvel[:] = tangent @ velocity
    mj.mj_forward(model, data)
    return vi, tangent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("folder", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--spring", choices=("installed", "removed"), required=True)
    args = ap.parse_args()
    print(json.dumps(prepare(args.folder, args.output, args.spring == "installed"), indent=2))


if __name__ == "__main__":
    main()
