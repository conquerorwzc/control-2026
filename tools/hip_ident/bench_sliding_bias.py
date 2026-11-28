"""Cross-validate sliding torque midpoints from training-only single-axis triangles."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose
from replay_spring_off_1128 import firmware_to_cad, geometry, read_segments


def collect(folder, side, zero, min_speed, max_acceleration):
    samples = defaultdict(lambda: defaultdict(list))
    for source, pose, trial, axis, _, _, _, _, rx, rows in read_segments(folder, side, (5,)):
        if pose == 6 or axis not in (0, 1) or trial != 5 or len(rows) <= 100:
            continue
        phase = (rows[:, 3].astype(int) >> 10) & 7
        grid = np.arange(rx[0].max(), rx[-1].min() + 1, 2000)
        q, v, tau = (np.empty((len(grid), 2)) for _ in range(3))
        for j in range(2):
            q[:, j] = np.interp(grid, rx[:, j], np.unwrap(firmware_to_cad(rows[:, 8 + 9 * j], zero[j])))
            v[:, j] = -np.interp(grid, rx[:, j], rows[:, 9 + 9 * j])
            tau[:, j] = -np.interp(grid, rx[:, j], rows[:, 12 + 9 * j])
        # Average speed over 50 ms before using a centered 80 ms derivative.
        v = np.column_stack([np.convolve(v[:, j], np.ones(25) / 25, mode="same") for j in range(2)])
        accel = (v[40:] - v[:-40]) / 0.080
        for i, acceleration in enumerate(accel, start=20):
            if i < 32 or i >= len(grid) - 32 or phase[np.searchsorted(rx[:, axis], grid[i])] != 2:
                continue
            speed = abs(v[i, axis])
            if speed < min_speed or max(abs(acceleration)) > max_acceleration or abs(v[i, 1 - axis]) > 0.08:
                continue
            key = tuple(np.round([q[i, axis] / 0.02, q[i, 1 - axis] / 0.02, speed / 0.08]).astype(int))
            samples[source, pose, axis][key].append((v[i, axis] > 0, q[i], v[i], tau[i]))
    return samples


def estimate(model, side, samples):
    joints = [side + "_front", side + "_rear"]
    data = mj.MjData(model)
    results = defaultdict(list)
    for (source, pose, axis), bins in samples.items():
        for values in bins.values():
            branches = [
                [(q, v, tau) for positive, q, v, tau in values if positive == sign] for sign in (False, True)
            ]
            if min(map(len, branches)) < 4:
                continue
            means = [np.mean(branch, axis=0) for branch in branches]
            if (
                np.max(abs(means[0][0] - means[1][0])) > 0.012
                or abs(means[0][1][axis] + means[1][1][axis]) > 0.045
            ):
                continue
            residuals = []
            for q, v, tau in means:
                mj.mj_resetData(model, data)
                _, tangent = set_pose(model, data, joints, q)
                residuals.append((tau - tangent.T @ data.qfrc_bias)[axis])
            negative, positive = residuals
            results[axis].append(
                dict(
                    source=source,
                    pose=pose,
                    midpoint_nm=(positive + negative) / 2,
                    half_difference_nm=(positive - negative) / 2,
                    speed_rad_s=float((means[1][1][axis] - means[0][1][axis]) / 2),
                )
            )
    return results


def summarize(rows):
    poses = sorted({r["pose"] for r in rows})
    errors = {p: np.array([r["midpoint_nm"] for r in rows if r["pose"] == p]) for p in poses}
    if len(poses) < 2:
        return {"pairs": len(rows), "poses": poses}
    means = {p: np.mean(x) for p, x in errors.items()}
    lop = {p: float(np.mean([means[q] for q in poses if q != p])) for p in poses}
    return dict(
        signed_midpoint_nm=float(np.mean(list(means.values()))),
        midpoint_range_nm=[float(min(means.values())), float(max(means.values()))],
        cad_rmse_nm=float(np.sqrt(np.mean([np.mean(x**2) for x in errors.values()]))),
        leave_one_pose_out_rmse_nm=float(
            np.sqrt(np.mean([np.mean((x - lop[p]) ** 2) for p, x in errors.items()]))
        ),
        per_pose={
            str(p): dict(
                pairs=len(x),
                midpoint_nm=float(means[p]),
                fitted_without_pose_nm=lop[p],
                cad_rmse_nm=float(np.sqrt(np.mean(x**2))),
                leave_one_pose_out_rmse_nm=float(np.sqrt(np.mean((x - lop[p]) ** 2))),
            )
            for p, x in errors.items()
        },
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("model", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--side", default="left", choices=("left", "right"))
    args = p.parse_args()
    model = mj.MjModel.from_xml_string(args.model.read_text(encoding="utf-8"))
    zero = geometry(model, args.side)
    variants = {}
    for minimum_speed, maximum_acceleration in (
        (0.2, 0.8),
        (0.2, 1.8),
        (0.1, 1.8),
        (0.3, 1.8),
    ):
        samples = collect(args.folder, args.side, zero, minimum_speed, maximum_acceleration)
        results = estimate(model, args.side, samples)
        variants[f"speed>={minimum_speed}, acceleration<={maximum_acceleration}"] = {
            name: dict(summary=summarize(results[axis]), bins=results[axis])
            for axis, name in enumerate(("front", "rear"))
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(dict(excluded_pose=6, side=args.side, variants=variants), indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
