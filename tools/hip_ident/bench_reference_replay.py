"""Replay spring-off bench trials using only the position reference after initialization."""

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose
from replay_spring_off_1128 import firmware_to_cad, read_segments, write_rows

STEP_US = 1000
POSE_HOLDOUT = 6
REFERENCE = [7, 16]
POSITION = [8, 17]
SPEED = [9, 18]
PRECLIP = [10, 19]
COMMAND = [11, 20]
TORQUE = [12, 21]


def wrap(q):
    return (q + np.pi) % (2 * np.pi) - np.pi


def board_us(rows):
    return rows[:, 1] * 65536 + rows[:, 2]


def drive_trace(issues, command, samples, initial, delay_us, time_constant_us):
    """Evaluate a delayed ZOH command followed by a unity-gain first-order lag."""
    shifted = issues + delay_us
    response = np.empty(len(shifted))
    response[0] = initial
    interval = np.diff(shifted)
    if time_constant_us == 0:
        response[:] = command
    else:
        alpha = np.exp(-interval / time_constant_us)
        for i, weight in enumerate(alpha):
            response[i + 1] = command[i] + weight * (response[i] - command[i])
    index = np.searchsorted(shifted, samples, side="right") - 1
    valid = index >= 0
    prediction = np.full(samples.shape, initial)
    k = index[valid]
    if time_constant_us == 0:
        prediction[valid] = response[k]
    else:
        weight = np.exp(-(samples[valid] - shifted[k]) / time_constant_us)
        prediction[valid] = command[k] + weight * (response[k] - command[k])
    return prediction


def fit_drive(segments):
    axes = [[], []]
    for _, pose, trial, axis, _, _, start, _, rx, rows in segments:
        if pose == POSE_HOLDOUT or trial != 3 or axis not in (0, 1):
            continue
        now = board_us(rows)
        issue = now + rows[:, 6]
        if issue[0] > start - 40_000 or issue[-1] < start + 120_000:
            continue
        select = (start - 40_000 <= now) & (now <= start + 120_000)
        issued = issue[select]
        observed = rx[select, axis]
        command = -rows[select, COMMAND[axis]]
        torque = -rows[select, TORQUE[axis]]
        scored = (start - 4000 <= observed) & (observed <= start + 100_000)
        axes[axis].append((issued, command, observed[scored], torque[scored], torque[0]))

    results = []
    for shots in axes:
        if not shots:
            raise ValueError("no contiguous training step edges for drive timing")

        def score(delay, time_constant):
            return float(
                np.sqrt(
                    np.mean(
                        [
                            np.mean((drive_trace(i, u, t, initial, delay, time_constant) - y) ** 2)
                            for i, u, t, y, initial in shots
                        ]
                    )
                )
            )

        candidates = [
            (score(delay, lag), delay, lag)
            for delay in range(0, 10_001, 500)
            for lag in range(0, 24_001, 1000)
        ]
        loss, delay, lag = min(candidates)
        results.append(
            dict(
                delay_us=delay,
                time_constant_us=lag,
                training_step_edges=len(shots),
                training_torque_rmse_nm=loss,
                training_instantaneous_torque_rmse_nm=score(0, 0),
            )
        )
    return results


def run_segment(model, data, joints, zero, segment, bias, drive, kp, kd, limit):
    _, _, _, _, _, _, _, _, rx, rows = segment
    issue = board_us(rows) + rows[:, 6]
    if np.any(np.diff(issue) <= 0):
        raise ValueError("nonmonotonic command publication timestamps")
    first = np.searchsorted(issue, rx[0].max(), side="left")
    if first >= len(issue):
        raise ValueError("command clock has no overlap with the feedback clock")
    start = issue[first]
    grid = np.arange(start, rx[-1].min() + 1, STEP_US)
    position = np.unwrap(firmware_to_cad(rows[:, POSITION], zero), axis=0)
    speed = -rows[:, SPEED]
    feedback = -rows[:, TORQUE]
    reference = firmware_to_cad(rows[first:, REFERENCE], zero)
    prior = np.array([np.searchsorted(rx[:, j], start, side="right") - 1 for j in range(2)])
    axes = np.arange(2)
    q0 = position[prior, axes]
    v0 = speed[prior, axes]
    initial_torque = feedback[prior, axes]
    mj.mj_resetData(model, data)
    vi, _ = set_pose(model, data, joints, q0, v0)
    qi = np.array([model.joint(j + "_sprocket_joint").qposadr[0] for j in joints])
    command = initial_torque.copy()
    lags = np.array([item["time_constant_us"] for item in drive])
    alpha = np.exp(-STEP_US / np.maximum(lags, 1))
    gain_mean = np.where(lags == 0, 0.0, lags / STEP_US * (1 - alpha))
    delays = np.array([item["delay_us"] for item in drive])
    # Publish on the first simulation grid point at or after the logged event.
    control_ticks = np.searchsorted(grid, issue[first:])
    arrival_ticks = np.searchsorted(grid, issue[first:, None] + delays)
    published = np.zeros_like(reference)
    actuator = initial_torque.copy()
    drive_target = initial_torque.copy()
    next_publish = 0
    next_arrival = [0, 0]
    trajectory = np.zeros((len(grid), 8))
    for i in range(len(grid)):
        while next_publish < len(control_ticks) and control_ticks[next_publish] == i:
            command = np.clip(
                bias + kp * wrap(reference[next_publish] - data.qpos[qi]) - kd * data.qvel[vi],
                -limit,
                limit,
            )
            published[next_publish] = command
            next_publish += 1
        for j in range(2):
            while next_arrival[j] < len(arrival_ticks) and arrival_ticks[next_arrival[j], j] == i:
                drive_target[j] = published[next_arrival[j], j]
                next_arrival[j] += 1
        mean_torque = drive_target + gain_mean * (actuator - drive_target)
        trajectory[i] = np.r_[
            data.qpos[qi],
            data.qvel[vi],
            np.where(lags == 0, drive_target, actuator),
            command,
        ]
        if i + 1 < len(grid):
            data.qfrc_applied[vi] = mean_torque
            mj.mj_step(model, data)
            actuator = drive_target + alpha * (actuator - drive_target)
    if np.any(data.warning.number) or not np.isfinite(trajectory).all():
        raise ValueError("reference replay diverged")
    observed = np.column_stack(
        [np.interp(grid, rx[:, j], signal[:, j]) for signal in (position, speed, feedback) for j in range(2)]
    )
    index = np.searchsorted(issue[first:], grid, side="right") - 1
    metrics = np.zeros((3, 2))
    for j in range(2):
        measured = (grid[0] < rx[:, j]) & (rx[:, j] <= grid[-1])
        for i, signal in enumerate((position, speed, feedback)):
            predicted = np.interp(rx[measured, j], grid, trajectory[:, 2 * i + j])
            metrics[i, j] = np.sqrt(np.mean((predicted - signal[measured, j]) ** 2))
    return np.column_stack((grid - start, reference[index], observed, trajectory)), metrics


def analyze_capture(captures, model_path, output):
    metadata = json.loads((model_path.parent / "parameters.json").read_text(encoding="utf-8"))
    model = mj.MjModel.from_xml_string(model_path.read_text(encoding="utf-8"))
    if (model.neq, model.nu, model.opt.timestep) != (4, 0, STEP_US / 1e6):
        raise ValueError("expected the 1 ms, four-closure, reduced fixed spring-off model")
    output.mkdir(parents=True, exist_ok=True)
    kp, kd, limit = (metadata[k] for k in ("pd_kp_nm_rad", "pd_kd_nm_s_rad", "torque_limit_nm"))
    summary = dict(
        model=str(model_path),
        shaft_armature_kg_m2=metadata["represented_cad_shaft_inertia_kg_m2"],
        additional_armature_kg_m2=metadata["additional_armature_kg_m2"],
        gravity_and_friction=metadata["legs"],
        pd_kp_nm_rad=kp,
        pd_kd_nm_s_rad=kd,
        torque_limit_nm=limit,
        internal_step_us=STEP_US,
        drive="Unity gain delayed ZOH followed by one first-order lag per axis, fit only on training step edges.",
        scope="A fixed-base spring-off position-control replay. CAN arrival is the observation timestamp used in the effective drive fit.",
        initialization="Most recent per-axis CAN state and drive torque at the first publication. Later gap-free fragments restart explicitly from known state with original baseline bias. No measured output enters later feedback.",
        bias="First known baseline record, recovered from logged preclip torque minus logged PD. Reused for later fragments of the same trial.",
        scoring="Equal weight per complete trial or per contiguous fragment, reported separately.",
        trajectory_columns=[
            "time_us",
            "ref_front",
            "ref_rear",
            "q_front",
            "q_rear",
            "speed_front",
            "speed_rear",
            "tau_front",
            "tau_rear",
            "sim_q_front",
            "sim_q_rear",
            "sim_speed_front",
            "sim_speed_rear",
            "sim_tau_front",
            "sim_tau_rear",
            "sim_cmd_front",
            "sim_cmd_rear",
        ],
        sides={},
    )
    metrics = []
    curves = {}
    for side in ("left", "right"):
        segments = list(read_segments(captures, side))
        drive = fit_drive(segments)
        instant = [dict(delay_us=0, time_constant_us=0) for _ in range(2)]
        summary["sides"][side] = dict(drive_fit=drive, scores={}, uninitializable_fragments=0)
        info = summary["sides"][side]
        joints = [side + "_front", side + "_rear"]
        zero = np.array(metadata["legs"][side]["cad_zero_in_fw_rad"])
        data = mj.MjData(model)
        biases = {}
        for segment in segments:
            source, pose, trial, axis, direction, row, start, whole, rx, rows = segment
            if pose != POSE_HOLDOUT:
                continue
            key = (source, pose, trial, axis, direction)
            now = board_us(rows)
            has_baseline = now[0] < start and ((int(rows[0, 3]) >> 10) & 7) == 1
            if key not in biases and has_baseline:
                error = wrap(rows[0, REFERENCE] - rows[0, POSITION])
                biases[key] = -(rows[0, PRECLIP] - kp * error + kd * rows[0, SPEED])
            if key not in biases or now[-1] < start:
                info["uninitializable_fragments"] += 1
                continue
            label = "whole" if whole else "baseline_prefix" if has_baseline else "restart"
            kind = "step" if trial == 3 else "sweep"
            result = dict(
                side=side,
                source=source,
                row=row,
                kind=kind,
                axis=axis,
                direction=direction,
                segment=label,
                start_relative_to_excite_us=int(now[0] - start),
                bias_front_nm=float(biases[key][0]),
                bias_rear_nm=float(biases[key][1]),
            )
            for name, timing in (("fitted", drive), ("instantaneous", instant)):
                curve, rmse = run_segment(
                    model,
                    data,
                    joints,
                    zero,
                    segment,
                    biases[key],
                    timing,
                    kp,
                    kd,
                    limit,
                )
                result["duration_s"] = curve[-1, 0] / 1e6
                for i, field in enumerate(("angle", "speed", "torque")):
                    for j, shaft in enumerate(("front", "rear")):
                        result[f"{name}_{field}_{shaft}_rmse"] = float(rmse[i, j])
                if name == "fitted":
                    curves[f"{side}_{source}_{row}"] = curve
            metrics.append(result)
        for kind in ("step", "sweep"):
            for label in ("whole", "baseline_prefix", "restart"):
                group = [
                    r for r in metrics if r["side"] == side and r["kind"] == kind and r["segment"] == label
                ]
                if group:
                    scores = {
                        k: float(np.sqrt(np.mean([r[k] ** 2 for r in group])))
                        for k in group[0]
                        if k.endswith("_rmse")
                    }
                    info["scores"][f"{kind}_{label}"] = dict(count=len(group), **scores)
        print(
            side,
            "drive",
            drive,
            "scored groups",
            {k: v["count"] for k, v in info["scores"].items()},
            flush=True,
        )
    write_rows(output / "replay_metrics.csv", metrics)
    np.savez_compressed(output / "holdout_trajectories.npz", **curves)
    (output / "parameters.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("captures", "model", "output"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    analyze_capture(args.captures, args.model, args.output)


if __name__ == "__main__":
    main()
