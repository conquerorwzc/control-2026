"""Offline joint-torque replay identification; see README.md before use."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


def load_csv(path, joints):
    a = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8-sig")
    names = ["time_s"] + [f"{j}_{s}" for s in ("q_rad", "qd_rad_s", "tau_cmd_nm") for j in joints]
    if a.ndim != 1 or len(a) < 2 or any(n not in a.dtype.names for n in names):
        raise ValueError(f"{path}: missing columns or fewer than two rows: {names}")
    x = np.column_stack([a[n] for n in names])
    if not np.isfinite(x).all() or not np.allclose(np.diff(x[:, 0]), .002, atol=1e-6, rtol=0):
        raise ValueError(f"{path}: nonfinite values or nonuniform 500 Hz timestamps")
    x[:, 0] -= x[0, 0]
    return x


def replay(cfg, x, p):
    m = mujoco.MjModel.from_xml_path(cfg["model"])
    if m.nu or np.any(m.jnt_type == mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError("Require fixed-base torque-replay model with no actuators")
    m.opt.timestep = .002
    ids = [m.joint(j).id for j in cfg["joints"]]
    if any(m.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE for i in ids):
        raise ValueError("Expected hinge joints")
    qi, vi = m.jnt_qposadr[ids], m.jnt_dofadr[ids]
    n = len(ids)
    m.dof_armature[vi] = p[:n]
    m.dof_frictionloss[vi] = p[n:2*n]
    m.dof_damping[vi] = p[2*n:3*n]
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key(cfg["initial_keyframe"]).id)
    d.qpos[qi], d.qvel[vi] = x[0, 1:1+n], x[0, 1+n:1+2*n]
    mujoco.mj_forward(m, d)
    out = np.empty((len(x), n))
    for k, t in enumerate(x[:, 0]):
        out[k] = d.qpos[qi]
        # ZOH applied command delay. Initial history must be recorded at steady state.
        idx = max(0, np.searchsorted(x[:, 0], t-p[-1], side="right")-1)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[vi] = x[idx, 1+2*n:] * p[3*n:4*n]
        if k+1 < len(x):
            mujoco.mj_step(m, d)
            if np.any(d.warning.number) or not np.isfinite(d.qpos).all():
                raise ValueError("MuJoCo warning/divergence; reject parameter bounds/model")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("config", type=Path)
    args = ap.parse_args()
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    if cfg["spring_installed"] is not False or len(cfg["joints"]) != 4:
        raise ValueError("This experiment requires four joints and no gas springs")
    bounds = np.asarray(cfg["bounds"], dtype=float)
    if bounds.shape != (17, 2) or not np.isfinite(bounds).all() or (bounds[:, 0] < 0).any() or (bounds[:, 1] < bounds[:, 0]).any():
        raise ValueError("bounds: 17 [lower, upper] pairs required, finite and nonnegative")
    train = load_csv(cfg["train_csv"], cfg["joints"])
    valid = load_csv(cfg["validation_csv"], cfg["joints"])
    if Path(cfg["train_csv"]).resolve() == Path(cfg["validation_csv"]).resolve():
        raise ValueError("Validation must be a separate experiment")
    rng = np.random.default_rng(cfg["seed"])
    best, loss = bounds.mean(axis=1), float("inf")
    for i in range(cfg["trials"]):
        candidate = best.copy() if i == 0 else rng.uniform(bounds[:, 0], bounds[:, 1])
        pred = replay(cfg, train, candidate)
        score = float(np.mean((pred-train[:, 1:5])**2))
        if score < loss:
            best, loss = candidate, score
    dest = Path(cfg["output"])
    dest.mkdir(parents=True, exist_ok=True)
    report = {"parameters": best.tolist(), "train_mse_rad2": loss, "config": cfg}
    for name, x in (("train", train), ("validation", valid)):
        pred = replay(cfg, x, best)
        report[name+"_rmse_rad"] = np.sqrt(np.mean((pred-x[:, 1:5])**2, axis=0)).tolist()
        np.savetxt(dest/(name+"_replay.csv"), np.column_stack((x[:, 0], x[:, 1:5], pred)), delimiter=",", header="time_s,"+",".join(f"{j}_{s}" for s in ("measured", "sim") for j in cfg["joints"]), comments="")
    (dest/"fit.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
