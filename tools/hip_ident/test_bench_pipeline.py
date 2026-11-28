"""Run with MuJoCo and the delivered CAD folder for an end-to-end numerical check."""

import csv
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

try:
    import mujoco as mj
except ImportError:
    mj = None


@unittest.skipUnless(mj, "requires MuJoCo")
class ReflectedInertiaTest(unittest.TestCase):
    def test_runtime_armature_uses_compiled_constraint_weights(self):
        from bench_model import set_armature

        xml = ('<mujoco><worldbody><body><inertial pos="0 0 0" mass="1" '
               'diaginertia=".001 .001 .001"/><joint type="hinge" frictionloss="1" '
               'armature="{}"/></body></worldbody></mujoco>')
        runtime = mj.MjModel.from_xml_string(xml.format(0))
        compiled = mj.MjModel.from_xml_string(xml.format(0.1))
        set_armature(runtime, mj.MjData(runtime), [0], [0.1])
        np.testing.assert_allclose(runtime.dof_invweight0, compiled.dof_invweight0)


class SweepQualityTest(unittest.TestCase):
    def test_variable_amplitude_and_clipped_window(self):
        from bench_data import sweep_tracking

        for duration in (20, 24):
            time = np.arange(-500, duration * 500 + 500) / 500
            hz = 0.5 + 4.5 * time / duration
            u = np.clip((hz - 3) / 2, 0, 1)
            amp = 0.05 * (1 + u**2 * (3 - 2 * u))
            angle = amp * np.sin(2 * np.pi * time * (0.5 + 2.25 * time / duration))
            ref = np.column_stack([angle, np.zeros_like(angle)])
            t = np.rint(time * 1e6).astype(int)
            shot = SimpleNamespace(
                t=t,
                tx=t,
                rx=np.column_stack([t, t]),
                phase=np.where(time < 0, 1, np.where(time < duration, 2, 3)),
                ref=ref,
                q=ref * 0.5,
                clipped=np.abs(hz - 2) < 0.015,
                fresh=np.ones_like(ref, dtype=bool),
                joints=["front", "rear"],
            )
            shot.q[time < 0] = 0
            rows = {row["hz"]: row for row in sweep_tracking(shot)}
            self.assertAlmostEqual(rows[4.6]["gain"], 0.5, places=5)
            self.assertGreater(rows[4.6]["reference_amp_rad"], 1.8 * rows[1.0]["reference_amp_rad"])
            self.assertTrue(rows[1.0]["usable_candidate"])
            self.assertFalse(rows[2.0]["usable_candidate"])


@unittest.skipUnless(mj and os.environ.get("WHEELLEG_CAD"), "requires MuJoCo and WHEELLEG_CAD")
class BenchTest(unittest.TestCase):
    def test_fixed_models_and_replay(self):
        from bench_model import prepare, set_pose, closure
        from extract_bench import extract
        from bench_replay import fit
        from bench_data import plateau_summary, Shot

        cad = Path(os.environ["WHEELLEG_CAD"])
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp)
            for stage in ("removed", "installed"):
                xml = dest / (stage + ".xml")
                result = prepare(cad, xml, stage == "installed")
                self.assertEqual(result["constraints"], 8 if stage == "removed" else 10)
                m = mj.MjModel.from_xml_path(str(xml))
                for side in ("left", "right"):
                    d = mj.MjData(m)
                    ids, b = set_pose(
                        m, d, [side + "_front", side + "_rear"], np.array([-0.3, 0.35]), np.zeros(2)
                    )
                    self.assertLess(max(abs(closure(m, d)[0])), 1e-7)
                    delta = (d.body(side + "_drive_wheel").xpos - d.body(side + "_hip").xpos)[[0, 2]]
                    self.assertAlmostEqual(np.linalg.norm(delta), 0.251452, places=5)
            model = mj.MjModel.from_xml_path(str(dest / "removed.xml"))
            for j in ("left_hip", "left_crank", "left_front_sprocket", "left_rear_sprocket"):
                model.dof_damping[model.joint(j + "_joint").dofadr[0]] = 0.0
            paths = []
            for idx, pose_id in enumerate((1, 6, 0)):
                raw = dest / f"trial{idx}.csv"
                data = mj.MjData(model)
                target = np.array([-0.3 - 0.02 * idx, 0.35])
                vi, b = set_pose(model, data, ["left_front", "left_rear"], target, np.zeros(2))
                qi = [model.joint(x + "_sprocket_joint").qposadr[0] for x in ("left_front", "left_rear")]
                bias = b.T @ data.qfrc_bias
                static_raw = dest / f"hold{idx}.csv"
                with raw.open("w", newline="") as f, static_raw.open("w", newline="") as sf:
                    w = csv.writer(f)
                    sw = csv.writer(sf)
                    for wr in (w, sw):
                        wr.writerow([f"I{i}" for i in range(25)])
                    for k in range(420):
                        t = k * 2000
                        phase = 5 if k < 2 else 1 if k < 70 else 2 if k < 320 else 3
                        cmd = bias + (0.12 * np.sin(0.1 * k) if phase == 2 else 0)
                        signals = []
                        for j in range(2):
                            signals.extend(
                                [
                                    target[j],
                                    data.qpos[qi[j]],
                                    data.qvel[vi[j]],
                                    cmd[j],
                                    cmd[j],
                                    cmd[j],
                                    2048,
                                    28,
                                    0,
                                ]
                            )
                        flags = 3 | 0xC0 | (3 << 12) | (3 << 18)
                        common = (3 << 17) | (1 << 15) | 4
                        w.writerow([k, t >> 16, t & 65535, common | (phase << 10), 256*pose_id+1, flags, 0, *signals])
                        sw.writerow(
                            [
                                k,
                                t >> 16,
                                t & 65535,
                                common | (9 << 4) | (1 << 10),
                                14,
                                flags,
                                0,
                                *([0, 0, 0, 0, 0, 0.3, 2048, 28, 0] * 2),
                            ]
                        )
                        if phase != 5:
                            # The fixture starts the simulated trial from its known closed baseline pose.
                            data.qfrc_applied[vi] = cmd
                            mj.mj_step(model, data, nstep=2)

                def subset(raw):
                    directory = dest / (raw.stem + "_segments")
                    reported = extract(raw, directory, "removed")
                    return directory / reported["segments"][0]["file"]

                paths.append(str(subset(raw)))
                self.assertEqual(Shot(paths[-1], "left").phase[0], 1)
                plateau = plateau_summary(Shot(subset(static_raw), "left"))
                self.assertTrue(plateau[0]["accepted"])
            cfg = dict(
                side="left",
                model="removed.xml",
                spring_installed=False,
                fw_rad_at_cad_zero=[0, 0],
                cad_sign=[1, 1],
                damping_nm_s_rad=[0, 0],
                coulomb_nm=[0, 0],
                armature_candidates_kg_m2=[[0], [0]],
                train_csvs=paths[:1],
                validation_csvs=[paths[1]],
                output="fit",
            )
            report = fit(cfg, dest)
            self.assertEqual(len(report["runs"]), 2)
            self.assertLess(max(report["runs"][1]["angle_rmse_rad"]), 0.005)
            for path in map(Path, paths):
                index = path.parent / "index.json"
                record = json.loads(index.read_text(encoding="utf-8"))
                record["source"] = str(dest / "same_capture.csv")
                index.write_text(json.dumps(record), encoding="utf-8")
            report = fit(dict(cfg, output="within_capture_holdout"), dest)
            self.assertEqual([r["workpoint_id"] for r in report["runs"]], [1, 6])
            report = fit(dict(cfg, train_csvs=[paths[2], paths[0]], output="combined_capture"), dest)
            self.assertEqual([r["workpoint_id"] for r in report["runs"]], [0, 1, 6])
            with self.assertRaisesRegex(ValueError, "separate original recording"):
                fit(dict(cfg, train_csvs=[paths[2]], validation_csvs=[paths[0]]), dest)
            with self.assertRaisesRegex(ValueError, "reserved for validation"):
                fit(dict(cfg, train_csvs=[paths[1]], validation_csvs=paths[:1]), dest)


if __name__ == "__main__":
    unittest.main()
