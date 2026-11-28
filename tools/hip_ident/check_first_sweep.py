"""Estimate reference tracking on the fixed, spring-removed 1127 rig. No hardware access."""

import argparse
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose


def wheel_xz(data):
    return (data.body("left_drive_wheel").xpos - data.body("left_hip").xpos)[[0, 2]]


def hanging_pose(model, joints):
    lo, hi = 0.0, 0.85
    for _ in range(24):
        half = (lo + hi) / 2
        data = mj.MjData(model)
        try:
            set_pose(model, data, joints, np.array([-half, half]))
        except ValueError:
            hi = half
            continue
        if np.linalg.norm(wheel_xz(data)) < 0.36486:
            lo = half
        else:
            hi = half
    data = mj.MjData(model)
    q = np.array([-lo, lo])
    set_pose(model, data, joints, q)
    x, z = wheel_xz(data)
    return q + np.arctan2(z, x) + np.pi / 2


def reference_delta(length):
    a, b = 0.210, 0.248
    return -2 * np.arccos((length**2 + a**2 - b**2) / (2 * length * a))


def free_hanging_tilt(model):
    # At the long stop, both shafts can still rotate together. Minimize CAD gravitational potential.
    joints = ["left_front", "left_rear"]
    q = hanging_pose(model, joints)
    lo, hi = -np.pi / 2, np.pi / 2
    for _ in range(22):
        angle = (lo + hi) / 2
        data = mj.MjData(model)
        _, tangent = set_pose(model, data, joints, q + angle)
        common_torque = np.sum(tangent.T @ data.qfrc_bias)
        if common_torque > 0:
            hi = angle
        else:
            lo = angle
    return np.rad2deg((lo + hi) / 2)


def run(model, amp, tilt_deg, freq_end=5.0, duration=24.0):
    joints = ["left_front", "left_rear"]
    qi = [model.joint(j + "_sprocket_joint").qposadr[0] for j in joints]
    data = mj.MjData(model)
    q0 = hanging_pose(model, joints) + np.deg2rad(tilt_deg)
    vi, _ = set_pose(model, data, joints, q0, np.zeros(2))
    # Both CAD shaft signs are assumed opposite to the firmware convention for this scenario.
    offset = (reference_delta(0.340) - reference_delta(0.36486)) * np.array([-0.5, 0.5])
    bias = np.zeros(2)
    settled_ticks = 0
    begin = None
    records = []
    clipped = False
    for k in range(int((10 + duration + 2) * 500)):
        t = k / 500
        u = min(1.0, t / 3.0)
        ref = q0 + offset * u**2 * (3.0 - 2.0 * u)
        length = np.linalg.norm(wheel_xz(data))
        q, v = data.qpos[qi], data.qvel[vi]
        x = -1 if begin is None else t - begin - 1
        if x >= duration + 1:
            break
        rate = (freq_end - 0.5) / duration
        phase = 2 * np.pi * x * (0.5 + 0.5 * rate * x)
        edge = np.clip(min(x, duration - x), 0.0, 1.0)
        ref[0] += amp * edge * np.sin(phase)
        if begin is None:
            if t >= 10:
                raise ValueError("simulated positioning timed out")
            bias = np.clip(bias + 0.002 * 16 * (ref - q), -2.0, 2.0)
            settled = u >= 1 and abs(length - 0.340) <= 0.007 and max(abs(v)) < 0.08 and not clipped
            settled_ticks = settled_ticks + 1 if settled else 0
            if settled_ticks >= 126:
                begin = t
        requested = bias + 10 * (ref - q) - 0.5 * v
        clipped = max(abs(requested)) > 2.0
        data.qfrc_applied[vi] = np.clip(requested, -2.0, 2.0)
        if 0 <= x < duration:
            records.append((x, phase, q[0], length, max(abs(requested)), clipped))
        mj.mj_step(model, data, nstep=2)
    r = np.array(records)
    gains = {}
    for hz in (1.0, 2.0, 3.0, 4.0, 4.6):
        t_mid = (hz - 0.5) / rate
        sub = r[(abs(r[:, 0] - t_mid) < 1) & (r[:, 0] >= 1) & (r[:, 0] < duration - 1)]
        if len(sub) < 750:
            continue
        design = np.column_stack((np.sin(sub[:, 1]), np.cos(sub[:, 1]), np.ones(len(sub)), sub[:, 0] - t_mid))
        gain = np.linalg.lstsq(design, sub[:, 2], rcond=None)[0][:2]
        gains[hz] = round(float(np.linalg.norm(gain) / amp), 3)
    print(
        f"A={amp:.3f} rad, tilt={tilt_deg:+g} deg, f_end={freq_end:g} Hz, positioning={begin:.2f} s, "
        f"requested_peak={max(r[:, 4]):.3f} Nm, clipped={100 * np.mean(r[:, 5]):.2f}%, "
        f"OW={1000 * min(r[:, 3]):.1f}..{1000 * max(r[:, 3]):.1f} mm, gains={gains}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    args = parser.parse_args()
    model = mj.MjModel.from_xml_path(str(args.model))
    if model.neq != 8 or model.nu or not np.isclose(model.opt.timestep, 0.001):
        raise ValueError("expected the 1 ms spring-removed fixed rig")
    tilt = free_hanging_tilt(model)
    print(f"CAD free-hanging equilibrium at the long stop: {tilt:+.2f} deg from wheel vertically below pivot")
    for amplitude in (0.025, 0.05):
        run(model, amplitude, tilt)
    for amplitude in (0.025, 0.05, 0.1):
        run(model, amplitude, 0)
    for tilt in (-15, 15):
        run(model, 0.025, tilt)
    run(model, 0.025, 0, freq_end=3)
