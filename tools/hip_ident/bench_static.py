"""Compare stable measured torque with CAD gravity at the measured pose, and match the later spring-on repeat."""

import argparse
import csv
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose


def holds(report, side, stage):
    if (report["spring"], report["side"]) != (stage, side):
        raise ValueError("spring stage or side mismatch")
    return [h for run in report["runs"] for h in run.get("holds", []) if h["accepted"]]


def values(h, suffix):
    return np.array([h["front_" + suffix], h["rear_" + suffix]])


def gravity(m, joints, q):
    d = mj.MjData(m)
    _, tangent = set_pose(m, d, joints, q, np.zeros(2))
    return tangent.T @ d.qfrc_bias, d, tangent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("config", type=Path, help="bench fit configuration with measured coordinate mapping")
    p.add_argument("removed", type=Path, help="bench_data JSON")
    p.add_argument("output", type=Path)
    p.add_argument("--installed", type=Path)
    p.add_argument("--installed-model", type=Path)
    p.add_argument("--match-rad", type=float, default=0.01)
    p.add_argument("--match-temp-c", type=float, default=3.0)
    args = p.parse_args()
    if bool(args.installed) != bool(args.installed_model):
        p.error("provide both installed inputs")
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    zero = np.asarray(cfg["fw_rad_at_cad_zero"], dtype=float)
    sign = np.asarray(cfg["cad_sign"], dtype=float)
    if (
        zero.shape != (2,)
        or not np.isfinite(zero).all()
        or sign.shape != (2,)
        or not np.all(np.abs(sign) == 1)
    ):
        raise ValueError("supply measured zero and signs")
    side = cfg["side"]
    joints = [side + "_front", side + "_rear"]
    base = holds(json.loads(args.removed.read_text()), side, "removed")
    model = mj.MjModel.from_xml_path(str(args.config.parent / cfg["model"]))
    if model.neq != 8 or model.nu:
        raise ValueError("expected spring-removed fixed bench")
    if args.installed:
        installed = holds(json.loads(args.installed.read_text()), side, "installed")
        loaded = mj.MjModel.from_xml_path(str(args.installed_model))
        if loaded.neq != 10 or loaded.nu:
            raise ValueError("expected spring-installed fixed bench")
    matched = set()
    rows = []
    for h in base:
        q = values(h, "q_rad")
        tau = values(h, "tau_nm") * sign
        g, _, _ = gravity(model, joints, (q - zero) * sign)
        row = dict(
            h,
            front_cad_gravity_nm=g[0],
            rear_cad_gravity_nm=g[1],
            front_residual_nm=tau[0] - g[0],
            rear_residual_nm=tau[1] - g[1],
        )
        if args.installed and h["approach"]:
            pairs = [
                (np.max(np.abs(values(k, "q_rad") - q)), idx, k)
                for idx, k in enumerate(installed)
                if idx not in matched
                and k["approach"] == h["approach"]
                and np.max(np.abs(values(k, "temp_c") - values(h, "temp_c"))) <= args.match_temp_c
            ]
            if pairs:
                dist, idx, k = min(pairs, key=lambda x: x[:2])
                if dist <= args.match_rad:
                    g2, d, tan = gravity(loaded, joints, (values(k, "q_rad") - zero) * sign)
                    slide = loaded.joint(side + "_spring_barrel_joint")
                    jac = tan[slide.dofadr[0]]
                    # Static motor + J_slide^T * axial force = gravity, in the CAD axis convention.
                    dtau = values(k, "tau_nm") * sign - tau
                    rhs = g2 - g - dtau
                    if np.linalg.norm(jac) > 1e-4:
                        force = float(jac @ rhs / (jac @ jac))
                        row.update(
                            slide_m=d.qpos[slide.qposadr[0]],
                            axial_force_n=force,
                            match_rad=dist,
                            fit_residual_nm=float(np.linalg.norm(rhs - jac * force)),
                        )
                        matched.add(idx)
        rows.append(row)
    if not rows:
        raise ValueError("no accepted static plateaus")
    with args.output.open("x", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        w.writeheader()
        w.writerows(rows)
    print("accepted no-spring holds", len(rows), "matched spring-on holds", len(matched))


if __name__ == "__main__":
    main()
