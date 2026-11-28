"""Split a raw VOFA recording at disabled rows, mode changes and missing samples."""

import argparse
import csv
import json
from pathlib import Path
from vofa_csv import SYSID_FIELDS, channel_indices, read_frame, scan


def extract(source, dest, spring):
    if dest.exists():
        raise FileExistsError(dest)
    dest.mkdir(parents=True)
    report = dict(source=str(source.resolve()), spring=spring, segments=[], invalid_rows=[], breaks=[])
    with source.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        idx = channel_indices(header, None, SYSID_FIELDS)
        block = []
        prev = None
        first = 0

        def finish():
            if not block:
                return
            info = {"start_row": first, "end_row": first + len(block) - 1}
            try:
                rows, _, fields, _ = scan(
                    iter(block), idx, len(header), 1, len(block), True, "sysid-hip", None
                )
            except ValueError as exc:
                info["rejected"] = str(exc)
            else:
                name = f"rows_{first}_{info['end_row']}.csv"
                with (dest / name).open("x", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(("time_s", *fields))
                    w.writerows(rows)
                info["file"] = name
            report["segments"].append(info)
            block.clear()

        for line, row in enumerate(reader, 1):
            try:
                if len(row) != len(header):
                    raise ValueError("wrong row width")
                f = read_frame(row, idx, SYSID_FIELDS)
            except (ValueError, IndexError) as exc:
                finish()
                prev = None
                report["invalid_rows"].append([line, str(exc)])
                continue
            mask = 3 << (2 * f.side)
            good = (
                f.version == 3
                and f.layout == 1
                and (f.state & 15) == 4
                and f.experiment in (0, 3, 4, 5, 9)
                and f.mode in (1, 2, 3, 8, 9, 10, 11, 14, 15)
                and (f.flags & 0x2C0) == 0xC0
                and f.applied == mask
                and f.valid & mask == mask
            )
            if (
                good
                and prev
                and (
                    f.run_key != prev.run_key
                    or (f.seq - prev.seq) % 65536 != 1
                    or not 0 < (f.timestamp - prev.timestamp) % 2**32 <= 3000
                )
            ):
                report["breaks"].append(dict(row=line, previous_seq=prev.seq, seq=f.seq))
                finish()
            if not good:
                finish()
                prev = None
                continue
            if not block:
                first = line
            block.append(row)
            prev = f
        finish()
    (dest / "index.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("source", type=Path)
    ap.add_argument("dest", type=Path)
    ap.add_argument("--spring", choices=("removed", "installed"), required=True)
    a = ap.parse_args()
    r = extract(a.source, a.dest, a.spring)
    print("exported", sum("file" in s for s in r["segments"]), "segments")
