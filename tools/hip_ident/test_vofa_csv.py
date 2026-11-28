import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest

import vofa_csv


class VofaCsvTest(unittest.TestCase):
    def convert(self, rows, *options, start=1, metadata=False):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "raw.csv"
            dest = Path(folder) / "processed.csv"
            sidecar = Path(folder) / "meta.json"
            if metadata:
                options = (*options, "--metadata-output", str(sidecar))
            with source.open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([f"CH{i}" for i in range(len(rows[0]))])
                writer.writerows(rows)
            log = io.StringIO()
            with contextlib.redirect_stderr(log), contextlib.redirect_stdout(log):
                code = vofa_csv.main([str(source), "--start-row", str(start), "--end-row", str(len(rows)),
                                      "--output", str(dest), *options])
            with dest.open(newline="") if dest.exists() else io.StringIO() as file:
                result = code, log.getvalue(), list(csv.DictReader(file))
            return (*result, json.loads(sidecar.read_text())) if metadata else result

    @staticmethod
    def packed(layout, experiment, *, side=0, pattern=0, phase=0, state=4):
        return (3 << 17) | (layout << 15) | (side << 14) | (phase << 10) | (pattern << 8) | (experiment << 4) | state

    @staticmethod
    def rows(state, mode, flags, width):
        # Both counters wrap at a valid 2 ms transition.
        times = ((0xffff, 0xffff, 0xfc18), (0, 0, 1000))
        return [[seq, hi, lo, state, mode, flags, *([0] * (width - 6))] for seq, hi, lo in times]

    def test_batch_boundaries_and_missing_frames(self):
        from extract_bench import extract

        stages = [(9, 15, 0, 1)]
        stages += [(4, 8, axis, direction) for axis in (0, 1) for direction in (1, -1)]
        stages += [(5, mode, axis, 1) for axis in range(4) for mode in (9, 10, 11)]
        stages += [(0, 1, axis, 1) for axis in range(4)]
        stages += [(3, 3, axis, direction) for axis in (0, 1) for direction in (1, -1)]
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "batch.csv"
            with src.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([f"I{i}" for i in range(25)])
                seq = 0
                for trial, mode, axis, direction in stages:
                    for phase in (1, 1, 2, 2, 3, 3):
                        if seq == 4:
                            seq += 1
                        packed = self.packed(1, trial, pattern=axis, phase=phase) | ((direction < 0) << 13)
                        writer.writerow([seq, (2000*seq) >> 16, (2000*seq) & 65535, packed, mode,
                                         0xc3 | (3<<12) | (3<<18), *([0]*19)])
                        seq += 1
            with contextlib.redirect_stderr(io.StringIO()):
                result = extract(src, Path(tmp)/"out", "removed")
            actual = [(s["experiment"], s["mode"], s["hip_pattern"], s["direction"]) for s in result["segments"]]
            self.assertEqual(actual, [stages[0], *stages])
            self.assertEqual(len(result["breaks"]), 1)
            self.assertTrue(all("file" in seg for seg in result["segments"]))

    def test_legacy_four_hip_csv(self):
        code, _, rows = self.convert(self.rows(4, 2, 0xef, 24))
        self.assertEqual(code, 0)
        self.assertEqual(rows[1]["time_s"], "0.002000")
        self.assertIn("right_rear_tau_cmd_nm", rows[1])
        self.assertNotIn("hip_pattern", rows[1])

    def test_single_leg_offsets_temperature_and_phase_change(self):
        capture = self.rows(self.packed(1, 0, side=1, pattern=3, phase=1), 1,
                            (0xc << 18) | (0xc << 12) | 0xcc, 25)
        capture[0][6], capture[0][14], capture[0][15] = 10, 36, -114
        capture[1][3] += 1 << 10
        code, _, rows = self.convert(capture, "--format", "sysid-hip")
        self.assertEqual(code, 0)
        self.assertEqual(rows[0]["hip_pattern"], "3")
        self.assertEqual(rows[0]["right_front_temperature_c"], "36.0")
        self.assertEqual(rows[0]["right_front_feedback_offset_us"], "-114.0")
        self.assertEqual(rows[0]["command_offset_us"], "10.0")
        self.assertEqual(rows[1]["phase"], "2")
        code, log, rows = self.convert(capture, "--format", "sysid-hip", "--required-mask", "0xf")
        self.assertEqual(code, 1)
        self.assertIn("missing-command", log)
        self.assertEqual(rows, [])

    def test_wheel_and_preserve_raw_feedback(self):
        capture = self.rows(self.packed(2, 8, phase=3), 4, (0x20 << 18) | (0x20 << 12) | 0xe0, 25)
        capture[0][20], capture[0][21], capture[0][24] = 0.125, -311, 1
        code, _, rows = self.convert(capture, "--format", "sysid-wheel")
        self.assertEqual(code, 0)
        self.assertEqual(rows[0]["right_wheel_current_cmd_a"], "0.125")
        self.assertEqual(rows[0]["right_wheel_current_fb_raw"], "-311.0")
        self.assertEqual(rows[0]["right_wheel_direct_current_mode"], "1.0")
        self.assertEqual(rows[0]["fresh_mask"], "32")
        capture[1][5] &= ~0x20
        code, log, rows = self.convert(capture, "--format", "sysid-wheel")
        self.assertEqual(code, 0)
        self.assertIn("stale-feedback", log)

    def test_sysid_cannot_be_read_as_legacy_or_swapped(self):
        capture = self.rows(self.packed(0, 0), 1, (0xf << 18) | (0xf << 12) | 0xcf, 25)
        for options in ((), ("--format", "sysid-wheel")):
            with self.subTest(options=options):
                code, log, _ = self.convert(capture, *options)
                self.assertEqual(code, 1)
                self.assertIn("format-mismatch", log)

    def test_four_joint_replay_and_firmware_metadata(self):
        active = self.rows(self.packed(0, 0), 1, (0xf << 18) | (0xf << 12) | 0xcf, 25)
        ready = self.rows(self.packed(3, 0, state=2), 1, 0x40, 25)
        ready[0][:3], ready[1][:3] = [0xfffd, 0xffff, 60536], [0xfffe, 0xffff, 62536]
        ready[0][6:11] = [0, 40000, 1234, 1, 0.2]
        ready[1][6:15] = [1, 0.4, 1, 13, 0.25, 9, -0.1, 1, 2]
        code, _, rows, metadata = self.convert(ready + active, "--format", "sysid-hip", "--required-mask", "0xf",
                                               start=3, metadata=True)
        self.assertEqual(code, 0)
        self.assertEqual(rows[0]["hip_temperature_joint"], "right_rear")
        self.assertIn("right_rear_tau_cmd_nm", rows[0])
        self.assertEqual(metadata["firmware"]["run"]["workpoint_id"], (40000 << 16) | 1234)
        self.assertEqual(metadata["firmware"]["left_front"]["kp_nm_per_rad"], 13)


if __name__ == "__main__":
    unittest.main()
