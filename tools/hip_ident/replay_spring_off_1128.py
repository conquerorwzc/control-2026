"""Fit shaft armature and replay the November 28 spring-off bench captures on the CAD closed chain."""

import argparse
import csv
import itertools
import json
from pathlib import Path

import mujoco as mj
from mujoco import rollout
import numpy as np

from bench_model import prepare, set_armature, set_pose

STEP = 0.001
WINDOW = 200
WRAP = 2 * np.pi
FULL = mj.mjtState.mjSTATE_FULLPHYSICS
FORCE = mj.mjtState.mjSTATE_QFRC_APPLIED


def records(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def write_rows(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def locate(folder, side):
    segments = []
    if side == "left":
        for r in records(folder / "1739_25项结果.csv"):
            segments.append(
                (
                    1739,
                    int(r["csv_row_start"]),
                    int(r["csv_row_end"]),
                    0,
                    int(r["trial"]),
                    int(r["axis_pattern"]),
                    int(r["direction"]),
                )
            )
        for r in records(folder / "1841_合并92项.csv"):
            if int(r["trial"]) not in (0, 3):
                continue
            segments.append(
                tuple(
                    int(r[k])
                    for k in ("source", "first_row", "last_row", "pose", "trial", "axis", "direction")
                )
            )
    else:
        for r in records(folder / "1939_117项.csv"):
            segments.append(
                (1939, *(int(r[k]) for k in ("start_row", "end_row", "pose", "trial", "axis", "direction")))
            )
    return sorted(s for s in segments if s[4] in (0, 3))


def geometry(model, side):
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    wheel = data.body(side + "_drive_wheel").xpos - data.body(side + "_hip").xpos
    knee = data.body(side + "_knee").xpos - data.body(side + "_hip").xpos
    # Firmware active-link axes use the rearward horizontal as zero and turn toward -Z.
    # CAD hinges turn about +Y. The front shaft is coupled 1:1 to the hip.
    front = np.arctan2(-knee[2], -knee[0])
    mean = np.arctan2(-wheel[2], -wheel[0])
    return np.array([front, 2 * mean - front])


def firmware_to_cad(q, zero):
    return (zero - q + np.pi) % WRAP - np.pi


def static_check(folder, model, side, joints, zero):
    filename = "1739_无簧静态网格.csv" if side == "left" else "1939_无簧静态网格.csv"
    data = mj.MjData(model)
    results = []
    for r in records(folder / filename):
        if r["accepted"] not in ("1", "True"):
            continue
        q = np.array([float(r[j + "_q_rad"]) for j in ("front", "rear")])
        _, jac = set_pose(model, data, joints, firmware_to_cad(q, zero))
        wheel = (data.body(side + "_drive_wheel").xpos - data.body(side + "_hip").xpos)[[0, 2]]
        front, rear = zero - firmware_to_cad(q, zero)
        half, mean = (front - rear) / 2, (front + rear) / 2
        length = 0.210 * np.cos(half) + np.sqrt(0.248**2 - (0.210 * np.sin(half)) ** 2)
        expected = -length * np.array([np.cos(mean), np.sin(mean)])
        torque = -np.array([float(r[j + "_tau_nm"]) for j in ("front", "rear")])
        results.append([np.linalg.norm(wheel - expected), *(jac.T @ data.qfrc_bias - torque)])
    a = np.array(results)
    return dict(
        holds=len(a),
        max_geometry_difference_mm=float(1000 * a[:, 0].max()),
        static_torque_rmse_including_friction_nm=np.sqrt(np.mean(a[:, 1:] ** 2, axis=0)).tolist(),
    )


def read_segments(folder, side):
    mask = 3 if side == "left" else 12
    for source, group in itertools.groupby(locate(folder, side), key=lambda s: s[0]):
        with (folder / f"{source}.csv").open("rb") as file:
            cursor = 1
            for _, first, last, pose, trial, axis, direction in group:
                for _ in itertools.islice(file, first - cursor):
                    pass
                a = np.loadtxt(itertools.islice(file, last - first + 1), delimiter=",", ndmin=2)
                cursor = last + 1
                state, flags = a[:, 3].astype(int), a[:, 5].astype(int)
                phase = (state >> 10) & 7
                relevant = (state % 16 == 4) & (phase != 5)
                if not np.all(
                    (a[relevant, 4].astype(int) >> 8 == pose)
                    & ((state[relevant] >> 4) & 15 == trial)
                    & ((state[relevant] >> 8) & 3 == axis)
                    & ((state[relevant] >> 14) & 1 == int(side == "right"))
                ):
                    raise ValueError(f"{source} rows {first}..{last}: index differs from logging metadata")
                good = relevant & ((flags & mask) == mask) & ((flags >> 18) == mask)
                good &= ((flags >> 12) & mask) == mask
                good &= (flags & 0x3C0) == 0xC0
                time = a[:, 1] * 65536 + a[:, 2]
                rx = time[:, None] + a[:, [15, 24]]
                delta = np.diff(time)
                gap = (np.diff(a[:, 0]) % 65536 != 1) | (delta <= 0) | (delta > 3000)
                gap |= np.any(np.diff(rx, axis=0) <= 0, axis=1)
                excite = np.flatnonzero(relevant & (phase == 2))
                if not len(excite):
                    continue
                start = time[excite[0]]
                edges = np.r_[0, 1 + np.flatnonzero(gap | (good[1:] != good[:-1])), len(a)]
                for lo, hi in zip(edges, edges[1:]):
                    if not good[lo] or hi - lo < 100:
                        continue
                    phases = phase[lo:hi]
                    whole = (
                        np.all(good[relevant]) and np.sum(relevant) == hi - lo and set(phases) >= {1, 2, 3}
                    )
                    yield (
                        source,
                        pose,
                        trial,
                        axis,
                        direction,
                        first + lo,
                        start,
                        whole,
                        rx[lo:hi],
                        a[lo:hi],
                    )


def windows(folder, model, side, joints, zero):
    local, whole = [], []
    for source, pose, trial, axis, direction, row, start, complete, rx, a in read_segments(folder, side):
        grid = np.arange(np.ceil(rx[0].max()), rx[-1].min() + 1, 1000)
        q = np.zeros((len(grid), 2))
        v, tau = q.copy(), q.copy()
        for j in range(2):
            t = rx[:, j]
            q[:, j] = np.interp(grid, t, np.unwrap(firmware_to_cad(a[:, 8 + 9 * j], zero[j])))
            v[:, j] = -np.interp(grid, t, a[:, 9 + 9 * j])
            tau[:, j] = -a[np.searchsorted(t, grid, side="right") - 1, 12 + 9 * j]

        def make(idx, size, label):
            data = mj.MjData(model)
            set_pose(model, data, joints, q[idx], v[idx])
            initial = np.empty(mj.mj_stateSize(model, FULL))
            mj.mj_getState(model, data, initial, FULL)
            return dict(
                source=source,
                row=int(row),
                pose=pose,
                trial=trial,
                axis=axis,
                direction=direction,
                label=label,
                initial=initial,
                q=q[idx : idx + size + 1],
                v=v[idx : idx + size + 1],
                tau=tau[idx : idx + size],
                first_time_us=float(grid[idx]),
            )

        if complete:
            whole.append(make(0, len(grid) - 1, "whole"))
        # Same excitation-relative times for both legs. Begin steps 10 ms before the edge.
        for sec in ([-0.010] if trial == 3 else [8.0, 16.0, 21.0]):
            target = start + sec * 1e6
            idx = np.searchsorted(grid, target)
            if idx < len(grid) - WINDOW and 0 <= grid[idx] - target <= 1000:
                local.append(make(idx, WINDOW, f"{sec:g}s"))
    return local, whole


def run(folder, output, model_path, side):
    model = mj.MjModel.from_xml_string(model_path.read_text(encoding="utf-8"))
    model.opt.timestep = STEP
    joints = [side + "_front", side + "_rear"]
    vi = [model.joint(j + "_sprocket_joint").dofadr[0] for j in joints]
    qi = [model.joint(j + "_sprocket_joint").qposadr[0] for j in joints]
    source = "1841_无簧左腿多姿态拟合.json" if side == "left" else "1939_无簧右腿拟合初值.json"
    friction = json.loads((folder / source).read_text(encoding="utf-8"))["moving_friction"]
    for j, i in zip(joints, vi):
        model.dof_damping[i] = friction[j]["damping_nm_s_rad"]
        model.dof_frictionloss[i] = friction[j]["coulomb_nm"]
    for part in ("hip", "crank"):
        model.dof_damping[model.joint(side + "_" + part + "_joint").dofadr] = 0
    zero = geometry(model, side)
    static = static_check(folder, model, side, joints, zero)
    local, whole = windows(folder, model, side, joints, zero)
    print(side, "geometry", zero, static, "windows", len(local), "complete", len(whole), flush=True)
    pool = [mj.MjData(model) for _ in range(4)]
    original = model.dof_armature[vi].copy()

    def batch(runs):
        control = np.zeros((len(runs), len(runs[0]["tau"]), model.nv))
        control[:, :, vi] = np.array([w["tau"] for w in runs])
        return np.stack([w["initial"] for w in runs]), control

    def predict(batch_input, arm):
        set_armature(model, pool[0], vi, original + arm)
        state, _ = rollout.rollout(
            model, pool, batch_input[0], batch_input[1], control_spec=int(FORCE), persistent_pool=True
        )
        if np.any(np.diff(state[:, :, 0], axis=1) <= 0) or not np.isfinite(state).all():
            raise ValueError("simulation stopped advancing")
        return state[:, :, 1 + np.array(qi)], state[:, :, 1 + model.nq + np.array(vi)]

    train = [w for w in local if w["pose"] != 6]
    validation = [w for w in local if w["pose"] == 6]
    if set(w["pose"] for w in train) != set(range(6)) or not validation:
        raise ValueError("incomplete pose split")
    train_batch = batch(train)
    q, v = (np.stack([w[key][1:] for w in train]) for key in ("q", "v"))
    scores = {}

    def evaluate(arm):
        key = tuple(np.round(arm, 9))
        if key not in scores:
            p, y = predict(train_batch, key)
            errors = np.mean(((p - q) / 0.03) ** 2 + ((y - v) / 0.4) ** 2, axis=(1, 2))
            scores[key] = [float(np.mean(errors[[w["pose"] == i for w in train]])) for i in range(6)]
        return np.mean(scores[key])

    # Diagnostic search bounds come from the scale of reflected CAD inertia at the training poses.
    data = mj.MjData(model)
    _, tangent = set_pose(model, data, joints, train[0]["q"][0])
    mass = np.zeros((model.nv, model.nv))
    mj.mj_fullM(model, data, mass)
    scale = np.diag(tangent.T @ mass @ tangent)
    bounds = scale * 4
    for iteration in range(3):
        grid = [np.r_[0.0, b * np.array([0.015625, 0.0625, 0.25, 1])] for b in bounds]
        best = np.array(min(itertools.product(*grid), key=evaluate))
        if not np.any(np.isclose(best, bounds)):
            break
        bounds = np.where(np.isclose(best, bounds), bounds * 4, bounds)
    for _ in range(4):
        axes = []
        for j in range(2):
            x = sorted(set(k[j] for k in scores))
            i = int(np.argmin(abs(np.array(x) - best[j])))
            axes.append(
                np.unique(
                    np.round(
                        np.r_[
                            np.linspace(x[max(0, i - 1)], x[i], 3),
                            np.linspace(x[i], x[min(len(x) - 1, i + 1)], 3),
                        ],
                        9,
                    )
                )
            )
        best = np.array(min([*scores, *itertools.product(*axes)], key=evaluate))
    print(side, "best", best, "scale", scale, flush=True)
    sensitivity = []
    for pose in range(6):
        sensitivity.append(min(scores, key=lambda k: np.mean(np.delete(scores[k], pose))))
    report_rows = []
    trajectories = {}

    def compare(runs):
        inp = batch(runs)
        cad = predict(inp, [0.0, 0.0])
        fitted = predict(inp, best)
        for n, w in enumerate(runs):
            meta = {
                k: w[k]
                for k in ("source", "row", "pose", "trial", "axis", "direction", "label", "first_time_us")
            }
            meta["held_out"] = w["pose"] == 6
            meta["duration_s"] = len(w["tau"]) * STEP
            for tag, (p, y) in (("cad", cad), ("fit", fitted)):
                for k, z in (("q", p[n]), ("v", y[n])):
                    error = np.sqrt(np.mean((z - w[k][1:]) ** 2, axis=0))
                    for j, axis in enumerate(("front", "rear")):
                        meta[f"{tag}_{k}_{axis}_rmse"] = float(error[j])
            meta["fit_front_error_at_500ms_rad"] = (
                float(fitted[0][n, 499, 0] - w["q"][500, 0]) if len(w["tau"]) >= 500 else None
            )
            meta["fit_rear_error_at_500ms_rad"] = (
                float(fitted[0][n, 499, 1] - w["q"][500, 1]) if len(w["tau"]) >= 500 else None
            )
            report_rows.append(meta)
            if meta["held_out"]:
                key = f"{w['source']}_{w['row']}_{w['label']}"
                trajectories[key] = np.column_stack(
                    [w["q"][1:], w["v"][1:], w["tau"], cad[0][n], cad[1][n], fitted[0][n], fitted[1][n]]
                )

    compare(train)
    compare(validation)
    for w in whole:
        compare([w])
    write_rows(output / (side + "_replay.csv"), report_rows)
    np.savez_compressed(output / (side + "_holdout_trajectories.npz"), **trajectories)

    def aggregate(group):
        return {
            key: np.sqrt(np.mean([r[key] ** 2 for r in group])).item()
            for key in group[0]
            if key.endswith("_rmse")
        }

    summaries = {}
    for held in (True, False):
        for long_run in (True, False):
            group = [r for r in report_rows if r["held_out"] == held and (r["label"] == "whole") == long_run]
            if group:
                summaries[f"{'held' if held else 'train'}_{'whole' if long_run else '200ms'}"] = dict(
                    count=len(group), **aggregate(group)
                )
    return dict(
        axis_order=joints,
        coordinate_source="CAD zero hip-knee and hip-wheel rays, negative +Y mapping; not independently surveyed",
        fw_at_cad_zero_rad=zero.tolist(),
        cad_sign=[-1, -1],
        geometry=static,
        friction=friction,
        cad_shaft_armature=original.tolist(),
        cad_projected_mass_diagonal=scale.tolist(),
        trajectory_columns=[
            "measured_q_front",
            "measured_q_rear",
            "measured_v_front",
            "measured_v_rear",
            "can_tau_front",
            "can_tau_rear",
            "cad_q_front",
            "cad_q_rear",
            "cad_v_front",
            "cad_v_rear",
            "fit_q_front",
            "fit_q_rear",
            "fit_v_front",
            "fit_v_rear",
        ],
        trajectory_frame="CAD shaft axes; torque held over the preceding 1 ms interval",
        training_search_upper_kg_m2=bounds.tolist(),
        additional_armature_kg_m2=best.tolist(),
        leave_one_training_pose_out_kg_m2=[list(v) for v in sensitivity],
        rmse=summaries,
        search=[dict(front=k[0], rear=k[1], loss_by_training_pose=v) for k, v in scores.items()],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("captures", "cad", "output"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = args.output / "bench_removed.xml"
    provenance = prepare(args.cad, model, False)
    for side in ("left", "right"):
        result = run(args.captures, args.output, model, side)
        result["source_model"] = provenance
        (args.output / (side + "_fit.json")).write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(side, result["rmse"], flush=True)


if __name__ == "__main__":
    main()
