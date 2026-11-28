"""Cross-check spring-off shaft armature with weak-form, local CAD inverse dynamics."""

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from numpy import hanning as hann

from bench_model import set_pose
from replay_spring_off_1128 import firmware_to_cad, geometry, read_segments, write_rows


def dynamics(model, joints, q):
    data = mj.MjData(model)
    _, tangent = set_pose(model, data, joints, q)
    mass = np.zeros((model.nv, model.nv))
    mj.mj_fullM(model, data, mass)
    return tangent.T @ mass @ tangent, tangent.T @ data.qfrc_bias


def interpolate(folder, side, zero):
    for source, pose, trial, axis, direction, row, start, _, rx, a in read_segments(folder, side):
        if trial != 0:
            continue
        grid = np.arange(np.ceil(rx[0].max()), rx[-1].min() + 1, 2000)
        t = (grid - start) * 1e-6
        q, v = (np.zeros((len(t), 2)) for _ in range(2))
        for j in range(2):
            q[:, j] = np.interp(grid, rx[:, j], np.unwrap(firmware_to_cad(a[:, 8 + 9 * j], zero[j])))
            v[:, j] = -np.interp(grid, rx[:, j], a[:, 9 + 9 * j])
        yield source, pose, axis, direction, row, t, q, v, (rx - start) * 1e-6, -a[:, [12, 21]]


def nonlinear_cad(model, joints, q, v, test, dt):
    """Project full CAD inverse dynamics without differentiating the measured speed."""
    data = mj.MjData(model)
    mass = np.zeros((model.nv, model.nv))
    momentum_map = np.empty((len(q), 2, model.nv))
    full_velocity = np.empty((len(q), model.nv))
    bias = np.empty((len(q), 2))
    for k, (qk, vk) in enumerate(zip(q, v)):
        _, tangent = set_pose(model, data, joints, qk, vk)
        mj.mj_fullM(model, data, mass)
        momentum_map[k] = tangent.T @ mass
        full_velocity[k] = data.qvel
        bias[k] = tangent.T @ data.qfrc_bias
    weighted_map = test[:, :, None, None] * momentum_map[:, None, :, :]
    weighted_derivative = np.gradient(weighted_map, dt, axis=0, edge_order=2)
    return test.T @ bias - np.einsum("tqjv,tv->qj", weighted_derivative, full_velocity)


def projections(folder, model, side):
    joints = [side + "_front", side + "_rear"]
    zero = geometry(model, side)
    for i in interpolate(folder, side, zero):
        source, pose, axis, direction, row, t, q, v, rx, tau = i
        for sec in (8, 12, 16, 19, 21):
            cut = (t >= sec - 1.5) & (t <= sec + 1.5)
            if np.count_nonzero(cut) < 1499:
                continue
            t1, q1, v1 = t[cut], q[cut], v[cut]
            # All timestamp-sensitivity probes must remain within this same
            # contiguous capture. Never wrap index -1 to its final sample.
            if t1[0] - 0.006 < rx[0].max() or t1[-1] + 0.006 > rx[-1].min():
                continue
            mass, _ = dynamics(model, joints, q1.mean(axis=0))
            q0 = q1.mean(axis=0)
            eps = 1e-4
            stiff = np.column_stack(
                [
                    (dynamics(model, joints, q0 + delta)[1] - dynamics(model, joints, q0 - delta)[1])
                    / (2 * eps)
                    for delta in np.eye(2) * eps
                ]
            )
            phase = 2 * np.pi * t1 * (0.5 + 0.5 * (4.5 / 24) * t1)
            test = hann(len(t1))[:, None] * np.column_stack((np.cos(phase), np.sin(phase)))
            # Remove constant and local linear torque drift while preserving zero endpoint value and slope.
            trend = np.column_stack((np.ones(len(t1)), t1 - sec))
            tapered_trend = trend * hann(len(t1))[:, None]
            test -= tapered_trend @ np.linalg.solve(trend.T @ tapered_trend, trend.T @ test)
            dt = 0.002
            norm = 2 / hann(len(t1)).sum()
            test_dt = np.gradient(test, dt, axis=0, edge_order=2)
            test_dt2 = np.gradient(test_dt, dt, axis=0, edge_order=2)
            aq = norm * test_dt2.T @ (q1 - q0)
            av = -norm * test_dt.T @ v1
            speed = norm * test.T @ v1
            dry = norm * test.T @ np.sign(v1)
            gravity = norm * test.T @ ((q1 - q0) @ stiff.T)
            exact = (2 / hann(len(t1))[::2].sum()) * nonlinear_cad(
                model, joints, q1[::2], v1[::2], test[::2], 2 * dt
            )
            angles = norm * test.T @ (q1 - q0)
            torques = {}
            for shift in (-0.006, -0.002, 0.0, 0.002, 0.006):
                # Positive shift samples later values to undo a possible feedback timestamp lag.
                sampled = np.column_stack(
                    [tau[np.searchsorted(rx[:, j], t1 + shift, side="right") - 1, j] for j in range(2)]
                )
                torques[shift] = norm * test.T @ sampled
            for j, joint in enumerate(joints):
                for p, quadrature in enumerate(("cos", "sin")):
                    result = dict(
                        side=side,
                        joint=joint,
                        source=source,
                        pose=pose,
                        axis=axis,
                        direction=direction,
                        row=row,
                        time_s=sec,
                        quadrature=quadrature,
                        angle_amplitude_rad=float(np.linalg.norm(angles[:, j])),
                        aq=aq[p, j],
                        av=av[p, j],
                        an=av[p, j],
                        speed=speed[p, j],
                        dry=dry[p, j],
                        cad_q=gravity[p, j] + (aq @ mass.T)[p, j],
                        cad_v=gravity[p, j] + (av @ mass.T)[p, j],
                        cad_n=exact[p, j],
                    )
                    result.update({f"torque_{shift}": values[p, j] for shift, values in torques.items()})
                    yield result


def fit(rows, a, shift, friction, fit_friction):
    x = np.array([[r["a" + a], r["speed"], r["dry"]] for r in rows]).reshape(-1, 3)
    y = np.array([r[f"torque_{shift}"] - r["cad_" + a] for r in rows])
    if not fit_friction:
        y -= x[:, 1:] @ friction
        x = x[:, :1]
    train = np.array([r["pose"] != 6 for r in rows])
    held = ~train
    coef = np.linalg.lstsq(x[train], y[train], rcond=None)[0]
    leave_out = [
        np.linalg.lstsq(x[train & mask], y[train & mask], rcond=None)[0][0]
        for p in range(6)
        if not np.all(mask := np.array([r["pose"] != p for r in rows]))
    ]
    prediction = x @ coef
    nominal = x[:, 1:] @ coef[1:]
    zero_coef = np.linalg.lstsq(x[train, 1:], y[train], rcond=None)[0]
    zero_refitted = x[:, 1:] @ zero_coef
    scores = np.stack(
        [
            x[mask].T @ (y[mask] - prediction[mask])
            for p in range(6)
            if np.any(mask := np.array([r["pose"] == p for r in rows]))
        ]
    )
    clusters, count, parameters = len(scores), int(train.sum()), x.shape[1]
    meat = scores.T @ scores
    bread = np.linalg.pinv(x[train].T @ x[train])
    cov = (clusters / (clusters - 1)) * ((count - 1) / (count - parameters)) * bread @ meat @ bread
    cluster_se = float(np.sqrt(max(cov[0, 0], 0.0)))
    return dict(
        estimate=coef.tolist(),
        leave_one_pose_out_kg_m2=leave_out,
        training_observations=int(train.sum()),
        pose_cluster_se_kg_m2=cluster_se,
        validation_observations=int(held.sum()),
        training_nm=float(np.sqrt(np.mean((y[train] - prediction[train]) ** 2))),
        validation_nm=float(np.sqrt(np.mean((y[held] - prediction[held]) ** 2))) if held.any() else None,
        validation_zero_addition_nm=(
            float(np.sqrt(np.mean((y[held] - nominal[held]) ** 2))) if held.any() else None
        ),
        validation_zero_addition_refit_nm=(
            float(np.sqrt(np.mean((y[held] - zero_refitted[held]) ** 2))) if held.any() else None
        ),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("captures", "bench_xml", "output"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = mj.MjModel.from_xml_string(args.bench_xml.read_text(encoding="utf-8"))
    for side, file in (("left", "1841_无簧左腿多姿态拟合.json"), ("right", "1939_无簧右腿拟合初值.json")):
        movement = json.loads((args.captures / file).read_text(encoding="utf-8"))["moving_friction"]
        all_rows = list(projections(args.captures, model, side))
        write_rows(args.output / f"{side}_projections.csv", all_rows)
        out = {}
        for joint, values in movement.items():
            friction = [values["damping_nm_s_rad"], values["coulomb_nm"]]
            out[joint] = {}
            for band, times in (("all", (8, 12, 16, 19, 21)), ("low", (8, 12)), ("high", (16, 19, 21))):
                rows = [
                    r
                    for r in all_rows
                    if r["joint"] == joint and r["time_s"] in times and r["angle_amplitude_rad"] >= 0.005
                ]
                out[joint][band] = {
                    f"{a}_{shift}_{free}": fit(rows, a, shift, friction, free)
                    for a in ("q", "v", "n")
                    for shift in (-0.006, -0.002, 0.0, 0.002, 0.006)
                    for free in (True, False)
                }
        (args.output / f"{side}_summary.json").write_text(json.dumps(out, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
