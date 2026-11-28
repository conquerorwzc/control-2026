"""Recheck the drift of the fixed-base 1128 spring-off bench under measured CAN torque."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
from mujoco import rollout
import numpy as np

from bench_hold_model import fit_hold_model
from bench_model import prepare, set_pose
from replay_spring_off_1128 import FORCE, geometry, windows, write_rows


def reduce_chains(xml):
    """Express each ideal 1:1 shaft and follower with one generalized coordinate."""
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    mass = np.empty((model.nv, model.nv))
    mj.mj_fullM(model, data, mass)
    tree = ET.fromstring(xml)
    eq = tree.find("equality")
    added = {}
    for side in ("left", "right"):
        for axis, part in (("front", "hip"), ("rear", "crank")):
            name = f"{side}_{axis}_sprocket"
            body = tree.find(f".//worldbody//body[@name='{name}']")
            shaft = body.find("joint")
            follower = tree.find(f".//worldbody//joint[@name='{side}_{part}_joint']")
            chain = eq.find(f"joint[@name='{name}_joint_chain']")
            if chain.get("joint1") != follower.get("name") or chain.get("joint2") != shaft.get("name"):
                raise ValueError("unexpected chain topology")
            if list(map(float, chain.get("polycoef").split())) != [0, 1, 0, 0, 0]:
                raise ValueError("only the 1:1 constant-ratio transmission is supported")
            dof = model.joint(shaft.get("name")).dofadr[0]
            added[name] = float(mass[dof, dof])
            follower.set("armature", str(float(follower.get("armature", 0)) + added[name]))
            # Keep the sprocket body's CAD mass fixed to this fixed bench. Its
            # center is on the shaft to within export tolerance; verify gravity below.
            eq.remove(chain)
            body.remove(shaft)
            follower.set("name", shaft.get("name"))
    return tree, added


def check_reduction(full, reduced, joints, poses):
    max_mass = max_gravity = max_wheel = 0.0
    for q in poses:

        def state(model):
            d = mj.MjData(model)
            _, t = set_pose(model, d, joints, q)
            M = np.empty((model.nv, model.nv))
            mj.mj_fullM(model, d, M)
            wheel = d.body(joints[0].split("_")[0] + "_drive_wheel").xpos
            return t.T @ M @ t, t.T @ d.qfrc_bias, wheel.copy()

        a, b = state(full), state(reduced)
        max_mass = max(max_mass, np.max(abs(a[0] - b[0])))
        max_gravity = max(max_gravity, np.max(abs(a[1] - b[1])))
        max_wheel = max(max_wheel, np.linalg.norm(a[2] - b[2]))
    if max_mass > 1e-6 or max_gravity > 1e-5 or max_wheel > 1e-6:
        raise ValueError(f"1:1 reduction mismatch: {max_mass}, {max_gravity}, {max_wheel}")
    return dict(max_mass_error_kg_m2=max_mass, max_gravity_error_nm=max_gravity, max_wheel_error_m=max_wheel)


def run(captures, cad, output):
    output.mkdir(exist_ok=True, parents=True)
    base_path = output / "cad_removed.xml"
    source = prepare(cad, base_path, False)
    xml = base_path.read_text(encoding="utf8")
    tree, polar = reduce_chains(xml)
    full = mj.MjModel.from_xml_string(xml)
    reduced = mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
    summary = dict(
        source=source,
        represented_cad_shaft_inertia_kg_m2=polar,
        additional_armature_kg_m2=[0] * 4,
        additional_armature_status="unresolved; CAD-only physical baseline",
        pd_kp_nm_rad=40,
        pd_kd_nm_s_rad=2,
        torque_limit_nm=40,
        can_torque_scale=1,
        legs={},
    )
    holds = {}
    for side in ("left", "right"):
        joints = [side + "_front", side + "_rear"]
        zero = geometry(full, side)
        holds[side] = fit_hold_model(captures, full, side, joints, zero)
        source_name = "1841_无簧左腿多姿态拟合.json" if side == "left" else "1939_无簧右腿拟合初值.json"
        moving = json.loads((captures / source_name).read_text(encoding="utf-8"))["moving_friction"]
        for j in joints:
            f = moving[j]
            node = tree.find(f".//worldbody//joint[@name='{j}_sprocket_joint']")
            node.set("damping", str(f["damping_nm_s_rad"]))
            node.set("frictionloss", str(f["coulomb_nm"]))
            # Numerical approximation of sticking at the measured 1 ms model timestep.
            node.set("solreffriction", ".002 1")
            node.set("solimpfriction", ".999 .999 .001 .5 2")
        summary["legs"][side] = dict(
            moving=moving, static=holds[side].report, cad_zero_in_fw_rad=zero.tolist(), cad_sign=[-1, -1]
        )
    nominal_xml = ET.tostring(tree, encoding="unicode")
    spec = mj.MjSpec.from_string(nominal_xml)
    for h in holds.values():
        h.configure_spec(spec)
    spec.compile()
    calibrated_xml = spec.to_xml()
    (output / "bench_spring_off.xml").write_text(calibrated_xml, encoding="utf8")
    models = [mj.MjModel.from_xml_string(x) for x in (nominal_xml, calibrated_xml)]
    analyses = []
    curves = {}
    for side in ("left", "right"):
        info = summary["legs"][side]
        joints = [side + "_front", side + "_rear"]
        data = [mj.MjData(m) for m in models]
        local, long = windows(captures, models[0], side, joints, np.array(info["cad_zero_in_fw_rad"]))
        info["reduction_check"] = check_reduction(full, reduced, joints, [w["q"][0] for w in local[::7]])
        vi = np.array([models[0].joint(j + "_sprocket_joint").dofadr[0] for j in joints])
        qi = np.array([models[0].joint(j + "_sprocket_joint").qposadr[0] for j in joints])
        for w in local + long:
            inputs = np.zeros((len(w["tau"]), models[0].nv))
            inputs[:, vi] = w["tau"]
            row = dict(
                side=side,
                **{
                    k: w[k]
                    for k in ("source", "row", "pose", "trial", "axis", "direction", "label", "first_time_us")
                },
            )
            row["n"] = len(inputs)
            row["held_out"] = w["pose"] == 6
            for tag, model, d in zip(("cad_gravity", "paired_gravity"), models, data):
                st, _ = rollout.rollout(model, d, w["initial"], inputs, control_spec=int(FORCE))
                if not np.all(np.isfinite(st)) or np.any(np.diff(st[0, :, 0]) <= 0):
                    raise ValueError(f"unstable replay: {row}")
                pred = st[0, :, 1 + qi].T
                vel = st[0, :, 1 + model.nq + vi].T
                e = pred - w["q"][1:]
                for j, axis in enumerate(("front", "rear")):
                    row[tag + "_angle_" + axis] = np.sqrt(np.mean(e[:, j] ** 2))
                    row[tag + "_speed_" + axis] = np.sqrt(np.mean((vel[:, j] - w["v"][1:, j]) ** 2))
                    if w["label"] == "whole":
                        row[tag + "_hold_500ms_" + axis] = abs(e[499, j])
                if tag == "paired_gravity" and row["held_out"]:
                    curves[f"{side}_{w['source']}_{w['row']}_{w['label']}"] = np.column_stack(
                        (w["q"][1:], pred, w["v"][1:], vel, w["tau"])
                    )
            analyses.append(row)
        for wname, cond in (("whole", True), ("windows_200ms", False)):
            subset = [
                r for r in analyses if r["side"] == side and r["held_out"] and (r["label"] == "whole") == cond
            ]
            info[wname] = dict(count=len(subset))
            for k in subset[0]:
                if "_angle_" in k or "_speed_" in k:
                    info[wname][k] = float(np.sqrt(np.mean([r[k] ** 2 for r in subset])))
                if "_hold_500ms_" in k:
                    info[wname][k] = max(r[k] for r in subset)
        print(side, info["whole"], info["windows_200ms"], flush=True)
    keys = list(dict.fromkeys(k for r in analyses for k in r))
    write_rows(output / "replay_metrics.csv", [{k: r.get(k, "") for k in keys} for r in analyses])
    np.savez_compressed(output / "holdout_trajectories.npz", **curves)
    summary["trajectory_columns"] = [
        "q_front",
        "q_rear",
        "predicted_q_front",
        "predicted_q_rear",
        "speed_front",
        "speed_rear",
        "predicted_speed_front",
        "predicted_speed_rear",
        "tau_front",
        "tau_rear",
    ]
    (output / "parameters.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf8"
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ("captures", "cad", "output"):
        ap.add_argument(key, type=Path)
    args = ap.parse_args()
    run(args.captures, args.cad, args.output)
