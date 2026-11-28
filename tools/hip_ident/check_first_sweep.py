"""Estimate reference tracking on the fixed, spring-removed 1127 rig. No hardware access."""

import argparse
from collections import deque
from pathlib import Path

import mujoco as mj
import numpy as np

from bench_model import set_pose

HIP_TORQUE_LIMIT_NM = 40.0  # motor_sysid per-hip max_torque_nm


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


def run(
    model,
    amp,
    tilt_deg,
    freq_end=5.0,
    duration=24.0,
    high_band=False,
    front_pd=(40.0, 2.0),
    delay_ticks=0,
    rear_pd=(40.0, 2.0),
):
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
    command_history = deque(np.zeros(2) for _ in range(delay_ticks))
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
        freq = 0.5 + rate * x
        u_amp = np.clip((freq - 3.0) / 2.0, 0.0, 1.0) if high_band else 0.0
        amplitude = amp * (1 + u_amp**2 * (3 - 2 * u_amp))
        ref[0] += amplitude * edge * np.sin(phase)
        positioning = begin is None
        if positioning:
            if t >= 10:
                raise ValueError("simulated positioning timed out")
            bias = np.clip(bias + 0.002 * 16 * (ref - q), -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM)
            settled = u >= 1 and abs(length - 0.340) <= 0.007 and max(abs(v)) < 0.08 and not clipped
            settled_ticks = settled_ticks + 1 if settled else 0
            if settled_ticks >= 126:
                begin = t
        requested = bias + 10 * (ref - q) - 0.5 * v
        if not positioning:
            kp, kd = front_pd
            requested[0] = bias[0] + kp * (ref[0] - q[0]) - kd * v[0]
            kp, kd = rear_pd
            requested[1] = bias[1] + kp * (ref[1] - q[1]) - kd * v[1]
        clipped = max(abs(requested)) > HIP_TORQUE_LIMIT_NM
        command_history.append(np.clip(requested, -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM))
        data.qfrc_applied[vi] = command_history.popleft()
        if 0 <= x < duration:
            records.append(
                (
                    x,
                    phase,
                    q[0],
                    length,
                    max(abs(requested)),
                    clipped,
                    ref[0],
                    max(abs(v)),
                    np.rad2deg(abs(ref[1] - q[1])),
                )
            )
        mj.mj_step(model, data, nstep=2)
    r = np.array(records)
    response = {}
    for hz in (1.0, 2.0, 3.0, 4.0, 4.6):
        t_mid = (hz - 0.5) / rate
        sub = r[(abs(r[:, 0] - t_mid) < 1) & (r[:, 0] >= 1) & (r[:, 0] < duration - 1)]
        if len(sub) < 750:
            continue
        design = np.column_stack((np.sin(sub[:, 1]), np.cos(sub[:, 1]), np.ones(len(sub)), sub[:, 0] - t_mid))
        coeff = np.linalg.lstsq(design, sub[:, [2, 6]], rcond=None)[0]
        output_amp, input_amp = np.linalg.norm(coeff[:2], axis=0)
        response[hz] = (round(float(output_amp), 4), round(float(output_amp / input_amp), 3))
    print(
        f"A={amp:.3f} rad, front_PD={front_pd}, delay_ms={2 * delay_ticks}, high_band={high_band}, tilt={tilt_deg:+g} deg, f_end={freq_end:g} Hz, positioning={begin:.2f} s, "
        f"requested_peak={max(r[:, 4]):.3f} Nm, clipped={100 * np.mean(r[:, 5]):.2f}%, "
        f"rear_PD={rear_pd}, rear_peak_error_deg={max(r[:, 8]):.2f}, "
        f"OW={1000 * min(r[:, 3]):.1f}..{1000 * max(r[:, 3]):.1f} mm, peak_speed={max(r[:, 7]):.3f} rad/s, "
        f"response_rad_and_gain={response}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    args = parser.parse_args()
    model = mj.MjModel.from_xml_path(str(args.model))
    if model.neq != 8 or model.nu or not np.isclose(model.opt.timestep, 0.001):
        raise ValueError("expected the 1 ms spring-removed fixed rig")
    tilt = free_hanging_tilt(model)
    run(model, 0.05, tilt, high_band=True, front_pd=(10.0, 0.5), rear_pd=(10.0, 0.5))
    run(model, 0.05, tilt, high_band=True, rear_pd=(10.0, 0.5))
    for delay_ticks in (6, 9):
        run(model, 0.05, tilt, high_band=True, delay_ticks=delay_ticks)
    print(f"CAD free-hanging equilibrium at the long stop: {tilt:+.2f} deg from wheel vertically below pivot")
    for angle in (tilt, -15, 0, 15):
        run(model, 0.05, angle, high_band=True)
    for amplitude in (0.025, 0.05):
        run(model, amplitude, tilt)
    for amplitude in (0.025, 0.05, 0.1):
        run(model, amplitude, 0)
    for tilt in (-15, 15):
        run(model, 0.05, tilt)
    run(model, 0.05, 0, freq_end=3)
