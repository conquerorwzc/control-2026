"""Replay independent, timestamped single-leg trials on a CAD-derived fixed rig."""

import json
from pathlib import Path
import mujoco as mj
import numpy as np

from bench_data import Shot
from bench_model import set_pose


def aligned(shot, zero, sign):
    mask = shot.fresh
    start = max(shot.rx[mask[:, j], j][0] for j in range(2))
    end = min(shot.rx[mask[:, j], j][-1] for j in range(2))
    times = np.arange(start, end + 1, 1000, dtype=np.int64)
    if len(times) < 3:
        raise ValueError("short feedback overlap")

    def sample(field, j):
        m = mask[:, j]
        return np.interp(times, shot.rx[m, j], field[m, j])

    q = np.column_stack([sample(shot.q, j) for j in range(2)])
    v = np.column_stack([sample(shot.v, j) for j in range(2)])
    torque = np.empty_like(q)
    for j in range(2):
        m = mask[:, j]
        ix = np.searchsorted(shot.rx[m, j], times, side="right") - 1
        torque[:, j] = shot.tau[m, j][ix] * sign[j]
    return times, (q - zero) * sign, v * sign, torque


def fit(cfg, folder):
    if cfg["spring_installed"] is not False:
        raise ValueError("fit motor parameters from spring-removed training shots")
    side = cfg["side"]
    if side not in ("left", "right"):
        raise ValueError("expected left or right")
    joints = [side + "_front", side + "_rear"]

    def pair(key):
        if any(x is None for x in cfg[key]):
            raise ValueError("supply measured " + key)
        x = np.asarray(cfg[key], dtype=float)
        if x.shape != (2,) or not np.isfinite(x).all():
            raise ValueError(key)
        return x

    zero, sign = pair("fw_rad_at_cad_zero"), pair("cad_sign")
    if not np.all(np.abs(sign) == 1):
        raise ValueError("cad_sign must be +/-1")
    model = mj.MjModel.from_xml_path(str(folder / cfg["model"]))
    if model.nu or model.neq != 8 or any(model.jnt_type == mj.mjtJoint.mjJNT_FREE):
        raise ValueError("use the fixed, removed-spring bench model")
    vi = np.array([model.joint(j + "_sprocket_joint").dofadr[0] for j in joints])
    qi = np.array([model.joint(j + "_sprocket_joint").qposadr[0] for j in joints])
    # Replace nominal CAD hip/crank and chain-driver damping with measured effective shaft values.
    followers = [model.joint(side + "_" + j + "_joint").dofadr[0] for j in ("hip", "crank")]
    model.dof_damping[followers] = 0.0
    damping, friction = pair("damping_nm_s_rad"), pair("coulomb_nm")
    if min(*damping, *friction) < 0:
        raise ValueError("negative identified friction")
    model.dof_damping[vi] = damping
    model.dof_frictionloss[vi] = friction
    model.opt.timestep = 0.001
    train = [(folder / p).resolve() for p in cfg["train_csvs"]]
    valid = [(folder / p).resolve() for p in cfg["validation_csvs"]]
    if not train or not valid or len(set(train + valid)) != len(train + valid):
        raise ValueError("supply distinct training and held-out recordings")
    windows = []
    sources = []
    for path in train + valid:
        s = Shot(path, side)
        if s.trial not in (0, 3) or not all(np.any(s.phase == phase) for phase in (1, 2, 3)):
            raise ValueError(f"{path}: need a continuous baseline, excitation and recovery")
        index = path.parent / "index.json"
        origin = path
        if index.exists():
            record = json.loads(index.read_text(encoding="utf-8"))
            if record["spring"] != "removed" or not any(
                seg.get("file") == path.name for seg in record["segments"]
            ):
                raise ValueError(f"{path}: capture index or spring stage mismatch")
            origin = Path(record["source"]).resolve()
        sources.append(origin)
        times, q, v, tau = aligned(s, zero, sign)
        data = mj.MjData(model)
        set_pose(model, data, joints, q[0], v[0])
        windows.append((data.qpos.copy(), data.qvel.copy(), times, q, v, tau))
    if set(sources[: len(train)]) & set(sources[len(train) :]):
        raise ValueError("use a separate original recording for validation")
    initial = model.dof_armature[vi].copy()
    candidates = [np.asarray(x, dtype=float) for x in cfg["armature_candidates_kg_m2"]]
    if len(candidates) != 2 or any(
        x.ndim != 1 or not x.size or not np.isfinite(x).all() or np.any(x < 0) for x in candidates
    ):
        raise ValueError("supply two finite nonnegative armature candidate lists")

    def replay(w, arm):
        initq, initv, times, q, v, tau = w
        data = mj.MjData(model)
        model.dof_armature[vi] = initial + arm
        data.qpos[:], data.qvel[:] = initq, initv
        mj.mj_forward(model, data)
        pos, vel = np.empty_like(q), np.empty_like(v)
        for k in range(len(times)):
            pos[k], vel[k] = data.qpos[qi], data.qvel[vi]
            if k + 1 < len(times):
                data.qfrc_applied[vi] = tau[k]
                mj.mj_step(model, data)
        if np.any(data.warning.number) or not np.isfinite(pos).all() or not np.isfinite(vel).all():
            raise ValueError("replay diverged")
        return pos, vel

    best = None
    for front in candidates[0]:
        for rear in candidates[1]:
            arm = np.array([front, rear])
            scores = []
            for w in windows[: len(train)]:
                _, _, _, q, v, _ = w
                pos, vel = replay(w, arm)
                # Equal trial weight, angle and velocity normalized by measured variation.
                scores.append(
                    np.mean(((pos - q) / np.maximum(np.std(q, axis=0), 0.025)) ** 2)
                    + np.mean(((vel - v) / np.maximum(np.std(v, axis=0), 0.1)) ** 2)
                )
            score = float(np.mean(scores))
            if best is None or score < best[0]:
                best = (score, arm)
    dest = folder / cfg["output"]
    dest.mkdir(parents=True, exist_ok=False)
    report = dict(
        spring="removed",
        torque_source="CAN feedback, unit scale",
        axis_order=joints,
        additional_armature_kg_m2=best[1].tolist(),
        train_score=best[0],
        runs=[],
    )
    for i, (path, w) in enumerate(zip(train + valid, windows)):
        _, _, times, q, v, tau = w
        p, y = replay(w, best[1])
        report["runs"].append(
            dict(
                source=str(path),
                held_out=i >= len(train),
                angle_rmse_rad=np.sqrt(np.mean((q - p) ** 2, axis=0)).tolist(),
                velocity_rmse_rad_s=np.sqrt(np.mean((v - y) ** 2, axis=0)).tolist(),
            )
        )
        np.savetxt(
            dest / f"replay_{i}.csv",
            np.column_stack((times, q, p, v, y, tau)),
            delimiter=",",
            header="time_us,q_f,q_r,sim_q_f,sim_q_r,v_f,v_r,sim_v_f,sim_v_r,tau_f,tau_r",
            comments="",
        )
    (dest / "fit.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report
