"""Identify spring-off bench gravity and static hysteresis from paired holds."""

from collections import defaultdict
import csv
from dataclasses import dataclass
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose


# Mechanically grouped CAD masses. The bound keeps the equivalent weights near
# their CAD values. It does not change the model's inertial mass or inertia.
BODY_GROUPS = (
    ("hip",),
    ("knee", "drive_wheel"),
    ("rear_link", "front_link", "parallel_link"),
    ("crank",),
)
MAX_WEIGHT_DELTA = 0.30


@dataclass
class HoldModel:
    body_ids: tuple
    weight_deltas: np.ndarray
    static_friction_nm: np.ndarray
    report: dict

    def configure_spec(self, spec):
        """Set gravity compensation on an uncompiled spec of the same bench."""
        # Negative factors increase gravity. MuJoCo only enables compensation
        # when a positive factor exists. The welded base has no generalized
        # coordinate, so its positive factor has no effect on bench dynamics.
        if np.any(self.weight_deltas > 0):
            base = spec.body("base_link")
            if base.parent.name != "world" or base.joints:
                raise ValueError("gravity correction requires the welded bench base")
            base.gravcomp = 1.0
        for names, delta in zip(self.report["gravity_body_names"], self.weight_deltas):
            for name in names:
                spec.body(name).gravcomp -= delta

    def gravity_correction(self, model, data):
        """Return the generalized force correction for the current forwarded CAD pose."""
        return -_gravity_basis(model, data, self.body_ids) @ self.weight_deltas


def _gravity_basis(model, data, groups):
    cols = np.zeros((model.nv, len(groups)))
    jac = np.empty((3, model.nv))
    for col, body_ids in enumerate(groups):
        for body in body_ids:
            mj.mj_jacBodyCom(model, data, jac, None, int(body))
            cols[:, col] -= model.body_mass[body] * (model.opt.gravity @ jac)
    return cols


def _pair_holds(path, zero):
    with path.open(encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))
    targets = defaultdict(lambda: defaultdict(list))
    accepted = 0
    for row in rows:
        if row["accepted"] not in ("1", "True"):
            continue
        accepted += 1
        ref = "ref" if "front_ref_rad" in row else "target"
        approach = row.get("approach_delta_sign", row.get("approach"))
        if approach not in ("-1", "1"):
            continue
        target = tuple(round(float(row[f"{axis}_{ref}_rad"]), 6) for axis in ("front", "rear"))
        q = np.array([float(row[f"{axis}_q_rad"]) for axis in ("front", "rear")])
        q = (zero - q + np.pi) % (2 * np.pi) - np.pi
        tau = -np.array([float(row[f"{axis}_tau_nm"]) for axis in ("front", "rear")])
        targets[target][int(approach)].append((q, tau))

    pairs = [(directions[-1], directions[1]) for directions in targets.values() if len(directions) == 2]
    return accepted, pairs


def _observations(model, joints, pairs, group_ids):
    data = mj.MjData(model)
    poses, residuals, features, widths = [], [], [], []
    for pair in pairs:
        visits = []
        for direction in pair:
            branch = []
            for q, tau in direction:
                mj.mj_resetData(model, data)
                _, tangent = set_pose(model, data, joints, q)
                cad = tangent.T @ data.qfrc_bias
                basis = tangent.T @ _gravity_basis(model, data, group_ids)
                branch.append((q, tau - cad, basis))
            visits.append(tuple(np.mean(a, axis=0) for a in zip(*branch)))
        q, residual, features_pair = (np.stack(a) for a in zip(*visits))
        poses.append(q)
        residuals.append(residual.mean(axis=0))
        features.append(features_pair.mean(axis=0))
        widths.append((residual[1] - residual[0], features_pair[1] - features_pair[0]))
    half_widths, width_features = zip(*widths)
    return tuple(map(np.asarray, (poses, residuals, features, half_widths, width_features)))


def fit_hold_model(folder, model, side, joints, zero):
    """Return the model and a JSON-ready report from the spring-off static grid.

    Each target contributes one pair, with the two approach directions weighted
    equally. Gravity is evaluated at each actual pose to remove gravity change
    due to different settling positions before estimating the hysteresis.
    """
    source = "1739" if side == "left" else "1939"
    (path,) = Path(folder).glob(f"{source}_*静态网格.csv")
    group_ids = tuple(np.array([model.body(f"{side}_{body}").id for body in group]) for group in BODY_GROUPS)
    accepted, pairs = _pair_holds(path, np.asarray(zero))
    if len(pairs) < 10:
        raise ValueError("insufficient bidirectional spring-off grid coverage")
    poses, residuals, features, width_residual, width_features = _observations(
        model, joints, pairs, group_ids
    )

    # Entire target pairs are held out, interleaved in grid acquisition order.
    validation = np.arange(len(pairs)) % 5 == 2
    training = ~validation
    selection, cv_rmse = _select_model(features[training], residuals[training])
    delta = _fit_candidate(selection, features[training], residuals[training])
    corrected = residuals - features @ delta
    # A paired residual span is a lower bound on the half width of the
    # breakaway envelope, not a separate friction term to add to moving friction.
    half_widths = 0.5 * np.abs(width_residual - width_features @ delta)
    friction = np.quantile(half_widths[training], 0.9, axis=0)
    report = dict(
        source=path.name,
        accepted_visits=accepted,
        paired_targets=len(pairs),
        training_targets=int(training.sum()),
        validation_targets=int(validation.sum()),
        cadence="Every fifth paired target in grid acquisition order held out before fitting",
        training_pose_min_rad=poses[training].min(axis=(0, 1)).tolist(),
        training_pose_max_rad=poses[training].max(axis=(0, 1)).tolist(),
        maximum_pair_settling_separation_rad=np.max(np.abs(poses[:, 1] - poses[:, 0]), axis=0).tolist(),
        gravity_body_names=[[f"{side}_{body}" for body in group] for group in BODY_GROUPS],
        gravity_selection=selection,
        training_cross_validation_rmse_nm=cv_rmse,
        gravity_weight_delta_bound=MAX_WEIGHT_DELTA,
        gravity_weight_deltas=delta.tolist(),
        training_cad_rmse_nm=_rmse(residuals[training]),
        training_calibrated_rmse_nm=_rmse(corrected[training]),
        validation_cad_rmse_nm=_rmse(residuals[validation]),
        validation_calibrated_rmse_nm=_rmse(corrected[validation]),
        training_static_hysteresis_p90_half_width_nm=friction.tolist(),
        validation_static_hysteresis_p90_half_width_nm=np.quantile(
            half_widths[validation], 0.9, axis=0
        ).tolist(),
    )
    return HoldModel(group_ids, delta, friction, report)


def _rmse(values):
    return np.sqrt(np.mean(values**2, axis=0)).tolist()


def _fit_weights(features, residuals):
    """Minimize soft-L1 torque error with coordinate-wise bounded updates."""
    x = features.reshape(-1, features.shape[-1])
    y = residuals.ravel()
    delta = np.zeros(x.shape[1])
    error = -y.copy()
    for _ in range(1000):
        previous = delta.copy()
        for j, column in enumerate(x.T):
            error -= column * delta[j]
            lo, hi = -MAX_WEIGHT_DELTA, MAX_WEIGHT_DELTA
            # The derivative is monotone in each coefficient.
            for _ in range(25):
                middle = (lo + hi) / 2
                trial = error + middle * column
                slope = column @ (trial / np.hypot(trial, 0.25))
                if slope > 0:
                    hi = middle
                else:
                    lo = middle
            delta[j] = (lo + hi) / 2
            error += column * delta[j]
        if np.max(np.abs(previous - delta)) < 1e-7:
            break
    return delta


def _fit_candidate(name, features, residuals):
    if name == "cad":
        return np.zeros(features.shape[-1])
    if name == "common_weight_scale":
        delta = _fit_weights(features.sum(axis=-1, keepdims=True), residuals)
        return np.repeat(delta, features.shape[-1])
    return _fit_weights(features, residuals)


def _select_model(features, residuals):
    """Select complexity using four folds within the fitting portion of the grid."""
    fold = np.arange(len(residuals)) % 4
    reports = {}
    selected = "cad"
    for name in ("cad", "common_weight_scale", "group_weight_scales"):
        errors = np.empty_like(residuals)
        for test in range(4):
            train = fold != test
            delta = _fit_candidate(name, features[train], residuals[train])
            errors[~train] = residuals[~train] - features[~train] @ delta
        reports[name] = float(np.sqrt(np.mean(errors**2)))
        if reports[name] < 0.99 * reports[selected]:
            selected = name
    return selected, reports
