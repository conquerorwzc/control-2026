"""Validate offline VOFA+ recordings and extract one motor identification segment."""

import argparse
from collections import Counter
import csv
from decimal import Decimal, InvalidOperation
import json
import math
from pathlib import Path
import sys
from typing import NamedTuple


JOINTS = ("left_front", "left_rear", "right_front", "right_rear")
SIGNALS = ("q_rad", "qd_rad_s", "tau_cmd_nm", "tau_fb_nm")
FIELDS = ("seq", "t_hi", "t_lo", "state", "mode", "flags") + tuple(
    f"{joint}_{signal}" for joint in JOINTS for signal in SIGNALS
) + ("q_ref0_rad", "q_ref1_rad")
CHANNELS = [f"CH{i}" for i in range(len(FIELDS))]
ACTUATORS = JOINTS + ("left_wheel", "right_wheel")
SYSID_FIELDS = FIELDS[:3] + ("packed_state", "mode", "flags") + tuple(f"CH{i}" for i in range(6, 25))
SINGLE_HIP_SIGNALS = ("q_ref_rad", "q_rad", "qd_rad_s", "tau_preclip_nm", "tau_cmd_nm", "tau_fb_nm",
                      "tau_fb_raw", "temperature_c", "feedback_offset_us")
WHEEL_SIGNALS = ("speed_ref_rad_s", "current_ref_a", "speed_rad_s", "current_preclip_a", "current_cmd_a",
                 "current_fb_raw", "temperature_c", "feedback_offset_us", "direct_current_mode")
EXPERIMENT_MODES = ((1, 2, 3), (4, 5, 6), (7,), (3,), (8,), (9, 10, 11), (12,), (4, 5, 6), (4, 5, 6))
CONFIG_NAMES = (
    ("workpoint_hi", "workpoint_lo", "direction", "selected_parameter", "real_hardware", "config_valid",
     "joint_mask", "wheel_mask"),
    ("center_rad", "configured", "kp_nm_per_rad", "kd_nm_per_rad_s", "max_torque_nm", "zero_rad",
     "motor_direction", "motor_per_joint"),
    ("configured", "kp_a_per_rad_s", "max_current_a", "motor_direction", "motor_per_wheel"),
    ("sweep_start_hz", "sweep_end_hz", "sweep_duration_s", "sweep_ramp_s", "step_baseline_s", "step_hold_s",
     "step_recovery_s", "steady_s", "coast_s", "hold_average_s", "motion_confirmation_s", "triangle_cycles",
     "hip_ramp_max_nm", "hip_motion_rad_s", "wheel_ramp_max_a", "wheel_motion_rad_s", "triangle_rad"),
)
NO_TIME_OFFSET = -16777216
SEQ_MOD = 1 << 16
TIME_MOD = 1 << 32
PERIOD_US = 2000
TOL_US = 1  # fit_real2sim.load_csv uses 0.002 s +/- 1e-6 s, rtol=0.


class Frame(NamedTuple):
    seq: int
    t_hi: int
    t_lo: int
    state: int
    mode: int
    flags: int
    signals: tuple

    @property
    def timestamp(self):
        return (self.t_hi << 16) | self.t_lo

    @property
    def version(self):
        return self.state >> 17

    @property
    def layout(self):
        return (self.state >> 15) & 3

    @property
    def experiment(self):
        return (self.state >> 4) & 15

    @property
    def valid(self):
        return (self.flags >> 12) & 0x3f

    @property
    def applied(self):
        return self.flags >> 18

    @property
    def side(self):
        return (self.state >> 14) & 1

    @property
    def run_key(self):
        # Phase may change within a run.
        return (self.state & ~(7 << 10), self.mode)


def channel_indices(header, map_path, fields=FIELDS):
    header = [name.strip() for name in header]
    names = ([f"CH{i}" for i in range(len(fields))] if map_path is None
             else json.loads(map_path.read_text(encoding="utf-8-sig")))
    if (not isinstance(names, list) or len(names) != len(fields)
            or any(not isinstance(name, str) or not name.strip() for name in names)):
        raise ValueError(f"channel map must contain {len(fields)} column names in channel order")
    names = [name.strip() for name in names]
    if len(set(names)) != len(names):
        raise ValueError("channel map names must be distinct")
    for name in names:
        if header.count(name) != 1:
            raise ValueError(f"expected exactly one CSV column named {name!r}; use --channel-map for custom headers")
    return [header.index(name) for name in names]


def read_frame(row, indices, fields=FIELDS):
    values = []
    for i, (name, index) in enumerate(zip(fields, indices)):
        cell = row[index]
        if i < 6:
            # Do not round fractional counters/flags to integers, even near a word boundary.
            try:
                value = Decimal(cell)
            except InvalidOperation:
                raise ValueError(f"{name}: not numeric ({cell!r})") from None
            limit = 0xFFFF if i < 3 else 0xFFFFFF  # Exact unsigned bitfields in float32.
            if not value.is_finite() or not 0 <= value <= limit or value != value.to_integral_value():
                raise ValueError(f"{name}: expected an integer in 0..{limit}, got {cell!r}")
            values.append(int(value))
        else:
            try:
                value = float(cell)
            except ValueError:
                raise ValueError(f"{name}: not numeric ({cell!r})") from None
            if not math.isfinite(value):
                raise ValueError(f"{name}: nonfinite value ({cell!r})")
            values.append(value)
    return Frame(*values[:6], tuple(values[6:]))


class Report:
    def __init__(self):
        self.counts = Counter()
        self.examples = {}
        self.failed = False

    def add(self, kind, location, detail, *, error=True):
        self.counts[kind] += 1
        self.examples.setdefault(kind, f"{location}: {detail}")
        self.failed |= error

    def show(self, label):
        print(label, file=sys.stderr)
        for kind, count in self.counts.items():
            print(f"  {kind}: {count}; first at {self.examples[kind]}", file=sys.stderr)


def seconds(us):
    return f"{us // 1_000_000}.{us % 1_000_000:06d}"


def config_page(frame):
    page = frame.signals[0]
    if not page.is_integer() or not 0 <= page <= 7:
        raise ValueError(f"invalid configuration page {page}")
    page = int(page)
    fields = CONFIG_NAMES[0 if page == 0 else 1 if page <= 4 else 2 if page <= 6 else 3]
    data = dict(zip(fields, frame.signals[1:]))
    if page == 0:
        hi, lo = data.pop("workpoint_hi"), data.pop("workpoint_lo")
        if not hi.is_integer() or not lo.is_integer() or not (0 <= hi <= 0xffff and 0 <= lo <= 0xffff):
            raise ValueError("invalid workpoint ID words")
        data["workpoint_id"] = (int(hi) << 16) | int(lo)
    elif page == 7:
        return "timing", data
    elif not data["configured"]:
        # A zero placeholder for missing hardware is not a calibration or gain.
        data = {"configured": False}
    return ("run" if page == 0 else ACTUATORS[page - 1]), data


def sysid_output_fields(layout, side):
    if layout == 0:
        signals = FIELDS[6:] + ("hip_temperature_c",)
    else:
        devices = JOINTS[2 * side:2 * side + 2] if layout == 1 else ACTUATORS[4:]
        names = SINGLE_HIP_SIGNALS if layout == 1 else WHEEL_SIGNALS
        signals = ("command_offset_us",) + tuple(f"{device}_{name}" for device in devices for name in names)
    metadata = ("state", "experiment", "hip_pattern", "phase", "direction", "fresh_mask", "valid_mask",
                "applied_mask")
    return SYSID_FIELDS[:6] + signals + metadata + (("hip_temperature_joint",) if layout == 0 else ())


def scan(reader, indices, width, start, end, keep_rows, capture_format, required_mask):
    legacy = capture_format == "legacy"
    fields = FIELDS if legacy else SYSID_FIELDS
    strict_cadence = legacy or required_mask == 0xf
    whole, selected = Report(), Report()
    output = []
    count = 0
    prev = None
    elapsed = 0
    profile = None
    pages = {}
    capture_metadata = {}
    ready_key = None
    for count, row in enumerate(reader, 1):
        in_range = count >= start and (end is None or count <= end)

        def note(kind, detail, *, edge=False, error=True):
            location = f"rows {count - 1}->{count}" if edge else f"row {count}"
            whole.add(kind, location, detail, error=error)
            if in_range and (not edge or count > start):
                selected.add(kind, location, detail, error=error)

        try:
            if len(row) != width:
                raise ValueError(f"expected {width} columns, got {len(row)}")
            frame = read_frame(row, indices, fields)
        except ValueError as exc:
            note("invalid-row", str(exc))
            prev = None
            continue

        config = not legacy and frame.version == 3 and frame.layout == 3
        if config:
            key = (frame.experiment, frame.mode, frame.state & 0x2300)
            if key != ready_key or prev is None or prev.layout != 3:
                pages = {}
                ready_key = key
            try:
                name, data = config_page(frame)
                pages[name] = {"data_row": count, **data}
            except ValueError as exc:
                note("bad-configuration", str(exc))
            if in_range:
                note("configuration-row", "select only active data rows")
        else:
            if not frame.flags & 0x80:
                note("execution-disabled", "flags bit 7 = 0 (dry run or execution interrupted)")
            if not frame.flags & 0x40:
                note("remote-offline", "flags bit 6 = 0")
        if legacy:
            if frame.state > 7:
                note("format-mismatch", "expected a legacy state; select --format sysid-hip or --format sysid-wheel")
            mask = 0xf
            if not frame.flags & 0x20:
                note("drive-not-ready", "flags bit 5 = 0")
        elif config:
            mask = 0
        else:
            hip = capture_format == "sysid-hip"
            experiments = (0, 3, 4, 5) if hip else (1, 2, 6, 7, 8)
            expected_layouts = (0, 1) if hip else (2,)
            if frame.version != 3 or frame.experiment not in experiments or frame.layout not in expected_layouts:
                note("format-mismatch",
                     f"version {frame.version}, experiment {frame.experiment}, layout {frame.layout}")
            accepted_modes = EXPERIMENT_MODES[frame.experiment] if frame.experiment < len(EXPERIMENT_MODES) else ()
            if frame.state & 0xf != 4 or frame.mode not in accepted_modes:
                note("not-running", f"state {frame.state & 0xf}, mode {frame.mode}")
            if fault := ((frame.flags >> 9) & 7):
                note("firmware-fault", f"fault code {fault}")
            expected_mask = ((0x3 << (2 * frame.side)) if frame.layout == 1 else 0xf) if hip else 0x30
            if not frame.applied or frame.applied & ~expected_mask:
                note("command-mask", f"unexpected applied mask {frame.applied:#04x}")
            mask = frame.applied if required_mask is None else required_mask
            if (frame.applied & mask) != mask:
                note("missing-command", f"required {mask:#04x}, applied {frame.applied:#04x}")
            if (frame.valid & mask) != mask:
                note("invalid-feedback", f"required {mask:#04x}, valid {frame.valid:#04x}")
            if prev is not None and prev.applied != frame.applied:
                note("command-mask-change", f"{prev.applied:#04x} -> {frame.applied:#04x}", edge=True)
            if frame.layout in (1, 2):
                offsets = [frame.signals[0]]
                first = 2 * frame.side if hip else 4
                column = 9 if hip else 8
                offsets.extend(frame.signals[column + 9 * i] for i in range(2) if mask & (1 << (first + i)))
                if any(not value.is_integer() or not NO_TIME_OFFSET < value < -NO_TIME_OFFSET for value in offsets):
                    note("invalid-time-offset", "missing or invalid command/feedback timestamp")
        if frame.flags & (0x10 if legacy else 0x100):
            note("command-clipped", "final command kept", error=False)
        stale = [name for bit, name in enumerate(ACTUATORS) if mask & ~frame.flags & (1 << bit)]
        if stale:
            note("stale-feedback", "no new feedback: " + ", ".join(stale), error=strict_cadence or frame.layout == 0)

        delta = None
        if prev is not None:
            current_key = (frame.state, frame.mode) if legacy else frame.run_key
            previous_key = (prev.state, prev.mode) if legacy else prev.run_key
            if current_key != previous_key:
                note("state-mode-change", f"({prev.state}, {prev.mode}) -> ({frame.state}, {frame.mode})", edge=True)
            step = (frame.seq - prev.seq) % SEQ_MOD
            seq_pair = f"seq {prev.seq} -> {frame.seq}"
            if step == 0:
                note("seq-duplicate", seq_pair, edge=True)
            elif step >= SEQ_MOD // 2:
                note("seq-backward-or-reset", f"{seq_pair}; forward distance ambiguous", edge=True)
            elif step > 1:
                note("seq-gap", f"{seq_pair}; {step - 1} missing if counter advanced normally", edge=True)

            delta = (frame.timestamp - prev.timestamp) % TIME_MOD
            time_pair = f"firmware timestamp {prev.timestamp} -> {frame.timestamp} us"
            if delta == 0:
                note("time-duplicate", time_pair, edge=True)
            elif delta >= TIME_MOD // 2:
                note("time-backward-or-reset", f"{time_pair}; forward distance ambiguous", edge=True)
            elif abs(delta - PERIOD_US) > TOL_US:
                kind = "time-gap" if delta > PERIOD_US + TOL_US else "time-cadence"
                note(kind, f"{time_pair}; delta {delta} us, expected {PERIOD_US} +/- {TOL_US} us", edge=True,
                     error=strict_cadence)
            elif frame.timestamp < prev.timestamp:
                note("time-wrap", f"{time_pair}; forward delta {delta} us", edge=True, error=False)

        if in_range:
            if delta is not None and count > start:
                prev_time = float(seconds(elapsed))
                elapsed += delta
                # Match the fitter's check on the actual serialized relative seconds, including float rounding.
                if (strict_cadence and abs(delta - PERIOD_US) <= TOL_US
                        and abs(float(seconds(elapsed)) - prev_time - 0.002) > 1e-6):
                    selected.add("fit-time-cadence", f"rows {count - 1}->{count}",
                                 f"delta {delta} us exceeds fitter tolerance after CSV-to-float conversion")
            if not legacy and not config and profile is None:
                profile = (frame.layout, frame.side)
                key = (frame.experiment, frame.mode, frame.state & 0x2300)
                capture_metadata = {"firmware": pages.copy() if key == ready_key else {}}
                capture_metadata.update(experiment=frame.experiment, mode=frame.mode,
                                        first_selected_data_row=start)
            if keep_rows and not config:
                if legacy:
                    output.append((seconds(elapsed), *frame[:6], *frame.signals))
                else:
                    metadata = (frame.state & 0xf, frame.experiment, (frame.state >> 8) & 3,
                                (frame.state >> 10) & 7, -1 if frame.state & (1 << 13) else 1,
                                frame.flags & 0x3f, frame.valid, frame.applied)
                    temperature = (ACTUATORS[frame.seq % 4],) if frame.layout == 0 else ()
                    output.append((seconds(elapsed), *frame[:6], *frame.signals, *metadata, *temperature))
        prev = frame

    whole.show(f"Capture: {count} data rows (1-based, header excluded)")
    last = count if end is None else end
    if last > count or last - start + 1 < 2:
        raise ValueError(f"requested range {start}..{last} must contain at least two rows within 1..{count}")
    selected.show(f"Selected rows {start}..{last} (inclusive)")
    if selected.failed:
        raise ValueError("selected range is not valid for real-data analysis; no CSV exported")
    return output, last - start + 1, FIELDS if legacy else sysid_output_fields(*profile), capture_metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="UTF-8 VOFA+ CSV export, with header")
    parser.add_argument("--channel-map", type=Path, help="JSON array of column names in channel order")
    parser.add_argument("--format", choices=("legacy", "sysid-hip", "sysid-wheel"), default="legacy")
    parser.add_argument("--required-mask", type=lambda value: int(value, 0),
                        help="required actuator mask, e.g. 0xf for four-hip replay")
    parser.add_argument("--start-row", type=int, help="first data row, 1-based and inclusive")
    parser.add_argument("--end-row", type=int, help="last data row, inclusive")
    parser.add_argument("--output", type=Path, help="new decoded CSV path; omit for check only")
    parser.add_argument("--metadata-output", type=Path, help="new JSON path for firmware configuration pages")
    args = parser.parse_args(argv)
    fields = FIELDS if args.format == "legacy" else SYSID_FIELDS
    if args.required_mask is not None:
        allowed = 0xf if args.format == "sysid-hip" else 0x30 if args.format == "sysid-wheel" else 0
        if args.required_mask <= 0 or args.required_mask & ~allowed:
            parser.error("--required-mask must match the selected sysid format")
    if args.metadata_output and (args.format == "legacy" or not args.output
                                 or args.metadata_output.resolve() == args.output.resolve()):
        parser.error("--metadata-output requires a sysid format and a separate --output CSV")
    if (args.start_row is None) != (args.end_row is None) or (args.output and args.start_row is None):
        parser.error("supply both --start-row and --end-row to select or export a range")
    if args.start_row is not None and not 1 <= args.start_row < args.end_row:
        parser.error("range must satisfy 1 <= start-row < end-row")

    try:
        with args.input.open(encoding="utf-8-sig", newline="") as source:
            reader = csv.reader(source, strict=True)
            header = next(reader, [])
            if not header:
                raise ValueError("missing CSV header")
            indices = channel_indices(header, args.channel_map, fields)
            rows, count, export_fields, metadata = scan(reader, indices, len(header), args.start_row or 1,
                                                        args.end_row, bool(args.output), args.format, args.required_mask)
        if args.metadata_output and args.metadata_output.exists():
            raise FileExistsError(args.metadata_output)
        if args.output:
            # Exclusive creation: do not overwrite an existing CSV (including the raw capture).
            with args.output.open("x", encoding="utf-8", newline="") as dest:
                writer = csv.writer(dest)
                writer.writerow(("time_s", *export_fields))
                writer.writerows(rows)
            print(f"Exported {count} rows to {args.output}")
            if args.metadata_output:
                with args.metadata_output.open("x", encoding="utf-8") as dest:
                    json.dump(metadata, dest, indent=2)
                    dest.write("\n")
        else:
            print(f"OK: {count} rows valid for {'replay' if args.format == 'legacy' else 'analysis'} (no output requested)")
    except (OSError, ValueError, csv.Error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
