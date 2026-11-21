"""Check offline VOFA+ CSV exports and extract one segment for torque replay."""

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


def channel_indices(header, map_path):
    header = [name.strip() for name in header]
    names = CHANNELS if map_path is None else json.loads(map_path.read_text(encoding="utf-8-sig"))
    if (not isinstance(names, list) or len(names) != len(FIELDS)
            or any(not isinstance(name, str) or not name.strip() for name in names)):
        raise ValueError("channel map must be a JSON array of 24 nonempty column names in CH0..CH23 order")
    names = [name.strip() for name in names]
    if len(set(names)) != len(names):
        raise ValueError("channel map names must be distinct")
    for name in names:
        if header.count(name) != 1:
            raise ValueError(f"expected exactly one CSV column named {name!r}; use --channel-map for custom headers")
    return [header.index(name) for name in names]


def read_frame(row, indices):
    values = []
    for i, (name, index) in enumerate(zip(FIELDS, indices)):
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


def scan(reader, indices, width, start, end, keep_rows):
    whole, selected = Report(), Report()
    output = []
    count = 0
    prev = None
    elapsed = 0
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
            frame = read_frame(row, indices)
        except ValueError as exc:
            note("invalid-row", str(exc))
            prev = None
            continue

        if not frame.flags & 0x80:
            note("execution-disabled", "flags bit 7 = 0 (dry run or execution interrupted)")
        if not frame.flags & 0x20:
            note("drive-not-ready", "flags bit 5 = 0")
        if not frame.flags & 0x40:
            note("remote-offline", "flags bit 6 = 0")
        if frame.flags & 0x10:
            note("command-clipped", "flags bit 4 = 1; final torque command kept", error=False)
        stale = [joint for bit, joint in enumerate(JOINTS) if not frame.flags & (1 << bit)]
        if stale:
            note("stale-feedback", "no new feedback: " + ", ".join(stale))

        delta = None
        if prev is not None:
            if (prev.state, prev.mode) != (frame.state, frame.mode):
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
                note(kind, f"{time_pair}; delta {delta} us, expected {PERIOD_US} +/- {TOL_US} us", edge=True)
            elif frame.timestamp < prev.timestamp:
                note("time-wrap", f"{time_pair}; forward delta {delta} us", edge=True, error=False)

        if in_range:
            if delta is not None and count > start:
                prev_time = float(seconds(elapsed))
                elapsed += delta
                # Match the fitter's check on the actual serialized relative seconds, including float rounding.
                if (abs(delta - PERIOD_US) <= TOL_US
                        and abs(float(seconds(elapsed)) - prev_time - 0.002) > 1e-6):
                    selected.add("fit-time-cadence", f"rows {count - 1}->{count}",
                                 f"delta {delta} us exceeds fitter tolerance after CSV-to-float conversion")
            if keep_rows:
                output.append((seconds(elapsed), *frame[:6], *frame.signals))
        prev = frame

    whole.show(f"Capture: {count} data rows (1-based, header excluded)")
    last = count if end is None else end
    if last > count or last - start + 1 < 2:
        raise ValueError(f"requested range {start}..{last} must contain at least two rows within 1..{count}")
    selected.show(f"Selected rows {start}..{last} (inclusive)")
    if selected.failed:
        raise ValueError("selected range is not valid for real-data replay; no CSV exported")
    return output, last - start + 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="UTF-8 VOFA+ CSV export, with header")
    parser.add_argument("--channel-map", type=Path, help="JSON array of column names, in CH0..CH23 order")
    parser.add_argument("--start-row", type=int, help="first data row, 1-based and inclusive")
    parser.add_argument("--end-row", type=int, help="last data row, inclusive")
    parser.add_argument("--output", type=Path, help="new fitter CSV path; omit for check only")
    args = parser.parse_args(argv)
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
            indices = channel_indices(header, args.channel_map)
            rows, count = scan(reader, indices, len(header), args.start_row or 1, args.end_row, bool(args.output))
        if args.output:
            # Exclusive creation: do not overwrite an existing CSV (including the raw capture).
            with args.output.open("x", encoding="utf-8", newline="") as dest:
                writer = csv.writer(dest)
                writer.writerow(("time_s", *FIELDS))
                writer.writerows(rows)
            print(f"Exported {count} rows to {args.output}")
        else:
            print(f"OK: {count} rows valid for replay (no output requested)")
    except (OSError, ValueError, csv.Error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
