"""Per-shot, single-leg processing for spring-off identification and later matched repeats."""

import argparse
import json
from pathlib import Path

import numpy as np


class Shot:
    def __init__(self, path, side):
        a = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8-sig"))
        self.side = side
        self.path = str(path)
        self.joints = [side + "_front", side + "_rear"]

        def matrix(name):
            return np.column_stack([a[j + "_" + name] for j in self.joints])

        self.q, self.v = matrix("q_rad"), matrix("qd_rad_s")
        self.tau, self.cmd = matrix("tau_fb_nm"), matrix("tau_cmd_nm")
        self.ref, self.temp = matrix("q_ref_rad"), matrix("temperature_c")
        self.t = np.rint(a["time_s"] * 1e6).astype(np.int64)
        self.tx = self.t + np.rint(a["command_offset_us"]).astype(np.int64)
        self.rx = self.t[:, None] + np.rint(matrix("feedback_offset_us")).astype(np.int64)
        bits = np.array([1, 2]) * (1 if side == "left" else 4)
        self.fresh = (a["fresh_mask"].astype(int)[:, None] & bits) != 0
        self.clipped = (a["flags"].astype(int) & 256) != 0
        self.phase = a["phase"].astype(int)
        self.trial = int(a["experiment"][0])
        self.mode = int(a["mode"][0])
        self.direction = int(a["direction"][0])
        self.pattern = int(a["hip_pattern"][0])
        if (
            len(a) < 2
            or not all(np.isfinite(a[k]).all() for k in a.dtype.names)
            or np.any(np.diff(a["seq"].astype(int)) % 65536 != 1)
            or np.any(np.diff(self.t) <= 0)
            or np.any(np.diff(self.tx) <= 0)
            or np.any(a["state"] != 4)
            or any(np.any(a[k] != a[k][0]) for k in ("mode", "experiment", "hip_pattern", "direction"))
            or np.any((a["valid_mask"].astype(int) & sum(bits)) != sum(bits))
            or np.any((a["applied_mask"].astype(int) & sum(bits)) != sum(bits))
        ):
            raise ValueError(f"{path}: expected one continuous enabled run with valid feedback")
        for j in range(2):
            times = self.rx[self.fresh[:, j], j]
            if len(times) < 2 or np.any(np.diff(times) <= 0):
                raise ValueError(f"{path}: invalid fresh feedback timestamps, axis {j}")


def plateau_summary(shot):
    if shot.trial != 9:
        raise ValueError("expected static hold/grid capture")
    stable = shot.phase == 1
    rows = []
    for ix in np.split(np.arange(len(stable)), np.flatnonzero(stable[1:] != stable[:-1]) + 1):
        if not stable[ix[0]]:
            continue
        tail = ix[shot.t[ix] >= shot.t[ix[-1]] - 500000]
        # Retain rejected visits in the report so a lost or unsettled pose is visible.
        ok = shot.t[tail[-1]] - shot.t[tail[0]] >= 450000 and not np.any(shot.clipped[tail])
        row = {"first_seq_time_s": shot.t[ix[0]] / 1e6, "accepted": False}
        before = np.searchsorted(shot.t, shot.t[ix[0]] - 150000)
        change = shot.ref[ix[0]] - shot.ref[before]
        row["approach"] = int(np.sign(change[0] - change[1]))
        for j, key in enumerate(("front", "rear")):
            fresh = tail[shot.fresh[tail, j]]
            if len(fresh) < 2:
                ok = False
                continue
            ok &= bool(np.ptp(shot.rx[fresh, j]) >= 440000)
            ok &= bool(np.max(abs(shot.v[fresh, j])) < 0.08)
            for label, field in (
                ("q_rad", shot.q),
                ("target_rad", shot.ref),
                ("tau_nm", shot.tau),
                ("cmd_nm", shot.cmd),
                ("temp_c", shot.temp),
            ):
                row[key + "_" + label] = float(np.mean(field[fresh, j]))
            row[key + "_tau_sd_nm"] = float(np.std(shot.tau[fresh, j]))
        row["accepted"] = bool(ok)
        rows.append(row)
    return rows


def response_delay(shot):
    # Relative publication-to-receipt delay, includes both the drive and the observation channel.
    results = []
    for j in range(2):
        use = shot.fresh[:, j] & (shot.phase == 2) & ~shot.clipped
        ts = shot.rx[use, j]
        measured = shot.tau[use, j]
        use2 = ts >= shot.tx[0] + 20000
        ts, measured = ts[use2], measured[use2]
        if len(ts) < 10 or np.std(measured) < 0.02:
            results.append(None)
            continue
        scores = []
        for lag in range(0, 20001, 1000):
            ix = np.searchsorted(shot.tx, ts - lag, side="right") - 1
            command = shot.cmd[ix, j]
            scores.append(float(np.corrcoef(command, measured)[0, 1]) if np.std(command) > 0.02 else -2.0)
        best = int(np.argmax(scores))
        results.append(
            dict(lag_ms=best, correlation=scores[best], boundary=best in (0, 20), correlations=scores)
            if scores[best] > -1
            else None
        )
    return results


def sweep_tracking(shot):
    excite = np.flatnonzero(shot.phase == 2)
    if not len(excite) or not np.any(shot.phase[: excite[0]] == 1):
        return []
    t0 = shot.t[excite[0]]
    recovery = np.flatnonzero(shot.phase == 3)
    if not len(recovery) or recovery[0] <= excite[-1]:
        return []
    duration_s = (shot.t[recovery[0]] - t0) * 1e-6
    rate = (5.0 - 0.5) / duration_s
    rows = []
    for j in range(2):
        baseline = (shot.phase == 1) & shot.fresh[:, j] & (shot.t >= t0 - 500000)
        baseline_rms = float(np.std(shot.q[baseline, j])) if baseline.sum() >= 20 else None
        for hz in (1.0, 2.0, 3.0, 4.0, 4.6):
            center = (hz - 0.5) / rate
            window = (shot.phase == 2) & (np.abs((shot.t - t0) * 1e-6 - center) <= 1.0)
            if not window.any():
                continue
            clipped_fraction = float(np.mean(shot.clipped[window]))

            def harmonic(times, signal, mask=None):
                t = (times - t0) * 1e-6
                use = (np.abs(t - center) <= 1.0) & (shot.phase == 2)
                if mask is not None:
                    use &= mask
                if use.sum() < 20 or np.ptp(t[use]) < 1.5:
                    return None
                phase = 2 * np.pi * t[use] * (0.5 + 0.5 * rate * t[use])
                design = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(phase)), t[use] - center))
                coef = np.linalg.lstsq(design, signal[use], rcond=None)[0]
                residual = signal[use] - design @ coef
                return complex(coef[0], coef[1]), float(np.sqrt(np.mean(residual**2)))

            ref = harmonic(shot.tx, shot.ref[:, j])
            angle = harmonic(shot.rx[:, j], shot.q[:, j], shot.fresh[:, j])
            if ref is None or angle is None or abs(ref[0]) < 0.005:
                continue
            gain = angle[0] / ref[0]
            output_rms = abs(angle[0]) / np.sqrt(2.0)
            floor = max(baseline_rms, angle[1]) if baseline_rms is not None else None
            # A quantized constant baseline is insufficient evidence for a numerical noise-floor estimate.
            ratio_db = float(20 * np.log10(output_rms / floor)) if floor and output_rms > 0 else None
            rows.append(
                dict(
                    joint=shot.joints[j],
                    hz=hz,
                    gain=abs(gain),
                    phase_deg=float(np.angle(gain, deg=True)),
                    reference_amp_rad=abs(ref[0]),
                    feedback_amp_rad=abs(angle[0]),
                    baseline_rms_rad=baseline_rms,
                    residual_rms_rad=angle[1],
                    signal_to_floor_db=ratio_db,
                    clipped_fraction=clipped_fraction,
                    usable_candidate=clipped_fraction == 0 and ratio_db is not None and ratio_db >= 20,
                )
            )
    return rows


def breakaway(shot):
    if shot.trial != 4 or shot.pattern not in (0, 1):
        raise ValueError("expected a single-axis torque ramp")
    j = shot.pattern
    baseline = np.flatnonzero((shot.phase == 1) & shot.fresh[:, j])
    if len(baseline) < 2:
        raise ValueError("missing holding-torque baseline")
    baseline = baseline[shot.t[baseline] >= shot.t[baseline[-1]] - 250000]
    hold = float(np.median(shot.tau[baseline, j]))
    run = np.flatnonzero((shot.phase == 2) & shot.fresh[:, j])
    start = None
    for i in run:
        if shot.clipped[i] or shot.v[i, j] * shot.direction < 0.08:
            start = None
            continue
        if start is None:
            start = i
        # Firmware exits on 100 ms; last active frame may precede that edge by one sample.
        if shot.rx[i, j] - shot.rx[start, j] >= 96000:
            return dict(
                axis=shot.joints[j],
                direction=shot.direction,
                hold_nm=hold,
                breakaway_increment_nm=float(shot.tau[start, j] - hold),
                command_at_onset_nm=float(shot.cmd[start, j]),
                onset_s=shot.t[start] / 1e6,
            )
    return dict(
        axis=shot.joints[j],
        direction=shot.direction,
        hold_nm=hold,
        breakaway_increment_nm=None,
        reason="no sustained onset in the selected samples",
    )


def sliding_friction(shots):
    # Pair constant-speed passes at matched *measured* two-joint positions.
    result = {}
    for j, key in enumerate(shots[0].joints):
        bins = {}
        for shot in shots:
            if shot.trial != 5 or shot.pattern != j:
                continue
            indices = np.flatnonzero(shot.fresh[:, j] & ~shot.clipped & (shot.phase == 2))
            if len(indices) < 3:
                continue
            v = shot.v[indices, j]
            for k in range(3, len(indices) - 3):
                i = indices[k]
                window = v[k - 3 : k + 4]
                if abs(v[k]) < 0.08 or np.ptp(window) > 0.09 or min(window) * max(window) <= 0:
                    continue
                # Do not pool distinct speeds, temperatures or the two directions.
                group = (shot.mode, *np.rint(shot.q[i] / 0.025).astype(int), round(shot.temp[i, j] / 3))
                bins.setdefault(group, [[], []])[int(v[k] > 0)].append((abs(v[k]), shot.tau[i, j]))
        pairs = []
        for neg, pos in bins.values():
            if min(len(neg), len(pos)) < 5:
                continue
            n, p = np.median(neg, axis=0), np.median(pos, axis=0)
            if abs(n[0] - p[0]) > 0.10:
                continue
            pairs.append(((p[0] + n[0]) / 2, (p[1] - n[1]) / 2))
        if len(pairs) >= 3 and np.ptp(np.asarray(pairs)[:, 0]) > 0.15:
            speed, torque = np.asarray(pairs).T
            design = np.column_stack((np.ones(len(speed)), speed))
            c, b = np.linalg.lstsq(design, torque, rcond=None)[0]
            result[key] = dict(
                paired_bins=len(speed),
                coulomb_nm=float(c),
                damping_nm_s_rad=float(b),
                rms_nm=float(np.sqrt(np.mean((torque - c - b * speed) ** 2))),
                physical=bool(c >= 0 and b >= 0),
            )
        else:
            result[key] = dict(
                paired_bins=len(pairs), reason="insufficient matched directions/speed coverage"
            )
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("side", choices=("left", "right"))
    ap.add_argument("inputs", nargs="+", type=Path)
    ap.add_argument("--spring", required=True, choices=("removed", "installed"))
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args()
    shots = [Shot(path, args.side) for path in args.inputs]
    for path in args.inputs:
        index = path.parent / "index.json"
        if index.is_file():
            record = json.loads(index.read_text(encoding="utf-8"))
            if record["spring"] != args.spring or not any(
                s.get("file") == path.name for s in record["segments"]
            ):
                raise ValueError(f"{path}: wrong spring stage or unlisted capture")
    report = {"spring": args.spring, "side": args.side, "torque_feedback_scale": 1.0, "runs": []}
    for s in shots:
        row = dict(
            source=s.path,
            experiment=s.trial,
            mode=s.mode,
            pattern=s.pattern,
            direction=s.direction,
            clipped_fraction=float(np.mean(s.clipped)),
            feedback_temperature_c=np.mean(s.temp, axis=0).tolist(),
        )
        if s.trial == 9:
            row["holds"] = plateau_summary(s)
            row["accepted_holds"] = sum(h["accepted"] for h in row["holds"])
        if s.trial == 4:
            row["onset"] = breakaway(s)
        if s.trial in (0, 3):
            row["publication_to_feedback"] = response_delay(s)
        if s.trial == 0:
            row["tracking"] = sweep_tracking(s)
        report["runs"].append(row)
    grids = {}
    for row in report["runs"]:
        if row["mode"] != 15:
            continue
        path = Path(row["source"])
        index = path.parent / "index.json"
        source = json.loads(index.read_text(encoding="utf-8"))["source"] if index.exists() else str(path)
        poses = grids.setdefault(source, {})
        for h in row["holds"]:
            if not h["accepted"] or not h["approach"]:
                continue
            key = round(h["front_target_rad"], 4), round(h["rear_target_rad"], 4)
            poses.setdefault(key, set()).add(h["approach"])
    report["grid_coverage"] = [
        dict(
            source=source,
            accepted_target_count=len(poses),
            bidirectional_pairs=sum(len(directions) == 2 for directions in poses.values()),
        )
        for source, poses in grids.items()
    ]
    if args.spring == "removed":
        report["moving_friction"] = sliding_friction(shots)
    with args.output.open("x", encoding="utf-8") as f:
        json.dump(report, f, indent=2, allow_nan=False)


if __name__ == "__main__":
    main()
