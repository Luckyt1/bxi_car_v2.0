"""Offline checks for arm torque CSV report generation."""

from __future__ import annotations

import csv
import fcntl
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


report = load_module("arm_torque_report_test", ROOT / "tools" / "arm_torque_report.py")


class ArmTorqueReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="arm torque report ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def csv_path(self, suffix: str = "000001-1234.csv") -> Path:
        return self.root / f"arm-torque-20261009-120000-{suffix}"

    def write_rows(self, path: Path, rows):
        fieldnames = [
            "elapsed_s",
            "wall_time_s",
            "motor1_permille",
            "motor1_rated_mnm",
            "motor2_permille",
            "motor2_rated_mnm",
            "motor3_permille",
            "motor3_rated_mnm",
            "motor4_permille",
            "motor4_rated_mnm",
            "dropped_samples",
        ]
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    def test_generates_png_and_summary_for_signed_torque(self):
        path = self.csv_path()
        self.write_rows(path, [
            {
                "elapsed_s": "0.0",
                "wall_time_s": "100.0",
                "motor1_permille": "500",
                "motor1_rated_mnm": "2000",
                "motor2_permille": "-250",
                "motor2_rated_mnm": "4000",
                "motor3_permille": "0",
                "motor3_rated_mnm": "3000",
                "motor4_permille": "100",
                "motor4_rated_mnm": "5000",
                "dropped_samples": "0",
            },
            {
                "elapsed_s": "1.0",
                "wall_time_s": "101.0",
                "motor1_permille": "-1000",
                "motor1_rated_mnm": "2000",
                "motor2_permille": "300",
                "motor2_rated_mnm": "4000",
                "motor3_permille": "200",
                "motor3_rated_mnm": "3000",
                "motor4_permille": "-400",
                "motor4_rated_mnm": "5000",
                "dropped_samples": "0",
            },
        ])

        image = report.generate_report(path)

        self.assertEqual(image, path.with_suffix(".png"))
        self.assertTrue(path.with_suffix(".png").is_file())
        summary = json.loads(path.with_suffix(".json").read_text())
        self.assertEqual(summary["generated_by"], report.GENERATED_BY)
        self.assertEqual(summary["motors"]["1"]["valid_samples"], 2)
        self.assertAlmostEqual(summary["motors"]["1"]["peak_abs"], -2.0)
        self.assertAlmostEqual(summary["motors"]["2"]["peak_negative"], -1.0)
        self.assertEqual(summary["dropped_samples"], 0)
        self.assertIn("motor-side", summary["note"])
        estimate = summary["output_estimate"]
        self.assertEqual(estimate["efficiency_assumed"], 1.0)
        self.assertFalse(estimate["transmission_losses_included"])
        self.assertIn("historical calibration", estimate["gear_ratio_basis"])
        self.assertAlmostEqual(estimate["motors"]["1"]["peak_abs"], -320.0)
        self.assertAlmostEqual(estimate["motors"]["4"]["peak_abs"], -202.0)

    def test_plot_scales_all_four_axes_and_labels_the_estimation_assumptions(self):
        path = self.csv_path()
        self.write_rows(path, [{
            "elapsed_s": "0.0",
            "motor1_permille": "-81", "motor1_rated_mnm": "3600",
            "motor2_permille": "-102", "motor2_rated_mnm": "3600",
            "motor3_permille": "-81", "motor3_rated_mnm": "1800",
            "motor4_permille": "10", "motor4_rated_mnm": "110",
        }])
        original_csv = path.read_bytes()
        plt = report.bootstrap_matplotlib()
        figure, axes = plt.subplots(4, 1)
        with mock.patch.object(plt, "subplots", return_value=(figure, axes)):
            report.generate_report(path)

        for axis, expected in zip(axes, [-46.656, -58.752, -23.328, 0.1111]):
            self.assertAlmostEqual(axis.lines[0].get_ydata()[0], expected)
            self.assertIn("output est.", axis.get_ylabel())
        figure_text = " ".join(text.get_text() for text in figure.texts)
        self.assertIn("transmission losses excluded", figure_text)
        self.assertIn("not freshly verified", figure_text)
        self.assertIn("Not measured output torque", figure_text)
        self.assertEqual(path.read_bytes(), original_csv)
        self.assertEqual(path.with_suffix(".png").stat().st_mode & 0o777, 0o644)
        self.assertEqual(path.with_suffix(".json").stat().st_mode & 0o777, 0o644)

    def test_old_motor_side_reports_are_regenerated_once_without_changing_csv(self):
        path = self.csv_path()
        self.write_rows(path, [{"elapsed_s": "0", "motor1_permille": "10", "motor1_rated_mnm": "1000"}])
        original_csv = path.read_bytes()
        report.generate_report(path)
        metadata_path = path.with_suffix(".json")
        metadata = json.loads(metadata_path.read_text())
        metadata.pop("report_version")
        metadata.pop("output_estimate")
        metadata_path.write_text(json.dumps(metadata))

        result = report.run(self.root, keep=3)

        self.assertEqual(result["generated"], [path.with_suffix(".png").name])
        self.assertEqual(result["failed"], [])
        self.assertEqual(path.read_bytes(), original_csv)
        self.assertTrue(report.generated_report_complete(path))
        self.assertEqual(report.run(self.root, keep=3)["generated"], [])
        with mock.patch.dict(report.INFERRED_GEAR_RATIOS, {1: 120}):
            self.assertFalse(report.generated_report_complete(path))

    def test_missing_values_and_incomplete_tail_are_not_counted_as_zero(self):
        path = self.csv_path()
        self.write_rows(path, [
            {
                "elapsed_s": "0.0",
                "wall_time_s": "100.0",
                "motor1_permille": "250",
                "motor1_rated_mnm": "2000",
                "motor2_permille": "",
                "motor2_rated_mnm": "4000",
                "motor3_permille": "",
                "motor3_rated_mnm": "",
                "motor4_permille": "bad",
                "motor4_rated_mnm": "5000",
                "dropped_samples": "3",
            },
        ])
        with path.open("a") as handle:
            handle.write("not-a-number,partial")

        report.generate_report(path)

        summary = json.loads(path.with_suffix(".json").read_text())
        self.assertEqual(summary["rows"], 1)
        self.assertEqual(summary["dropped_samples"], 3)
        self.assertEqual(summary["motors"]["1"]["valid_samples"], 1)
        self.assertEqual(summary["motors"]["2"]["valid_samples"], 0)
        self.assertIsNone(summary["motors"]["2"]["peak_abs"])

    def test_invalid_rated_torque_does_not_fake_nm_and_unit_stays_consistent(self):
        path = self.csv_path()
        self.write_rows(path, [
            {
                "elapsed_s": "0.0",
                "motor1_permille": "10",
                "motor1_rated_mnm": "-100",
                "motor2_permille": "20",
                "motor2_rated_mnm": "0",
            },
            {
                "elapsed_s": "1.0",
                "motor1_permille": "30",
                "motor1_rated_mnm": "2000",
                "motor2_permille": "40",
                "motor2_rated_mnm": "5000",
            },
        ])

        summary = report.summarize(path)
        series = report.collect_plot_series(path, summary.rows, report.choose_motor_units(path))

        self.assertEqual(summary.motors["1"]["unit"], "permille")
        self.assertEqual(summary.motors["1"]["valid_samples"], 2)
        self.assertEqual(summary.motors["1"]["peak_abs"], 30.0)
        self.assertEqual(summary.motors["2"]["unit"], "permille")
        self.assertEqual(series[1][2], "permille")
        self.assertEqual([value for value in series[1][1] if not math.isnan(value)], [10.0, 30.0])
        self.assertFalse(summary.output_estimate(1)["available"])
        self.assertIsNone(summary.output_estimate(1)["peak_abs"])
        plt = report.bootstrap_matplotlib()
        figure, axes = plt.subplots(4, 1)
        with mock.patch.object(plt, "subplots", return_value=(figure, axes)):
            report.generate_report(path)
        self.assertEqual(list(axes[0].lines[0].get_ydata()), [10.0, 30.0])
        self.assertIn("permille", axes[0].get_ylabel())
        self.assertIn("Output estimate unavailable", axes[0].texts[0].get_text())

    def test_downsampled_bucket_gap_does_not_connect_extremes_across_missing_row(self):
        bucket = report.Bucket()
        bucket.add(0.0, 1.0)
        bucket.add(0.5, None)
        bucket.add(1.0, -1.0)

        points = bucket.points()

        self.assertEqual(points[0][0], 0.0)
        self.assertTrue(math.isnan(points[0][1]))
        self.assertEqual(points[1], (0.0, 1.0))
        self.assertTrue(math.isnan(points[2][1]))
        self.assertEqual(points[3][0], 1.0)
        self.assertTrue(math.isnan(points[3][1]))
        self.assertEqual(points[4], (1.0, -1.0))
        self.assertTrue(math.isnan(points[5][1]))

    def test_keep_latest_three_deletes_only_owned_generated_reports(self):
        paths = []
        for second in range(5):
            path = self.root / f"arm-torque-20261009-12000{second}-000001-1234.csv"
            self.write_rows(path, [{
                "elapsed_s": "0.0",
                "wall_time_s": "1.0",
                "motor1_permille": "1",
                "motor1_rated_mnm": "1000",
                "dropped_samples": "0",
            }])
            report.write_summary(path, report.summarize(path), path.with_suffix(".png"))
            path.with_suffix(".png").write_bytes(b"generated")
            paths.append(path)
        unknown = paths[0].with_name("not-arm-torque-20261009-120000-000001-1234.csv")
        unknown.write_text("leave me")
        protected_png = paths[1].with_suffix(".png")
        paths[1].with_suffix(".json").write_text('{"generated_by": "somebody else"}\n')
        protected_png.write_text("leave me too")

        removed = report.cleanup_old_reports(self.root, keep=3)

        remaining_csv = sorted(path.name for path in report.owned_completed_csvs(self.root))
        self.assertEqual(remaining_csv, sorted(path.name for path in paths[2:]))
        self.assertTrue(unknown.exists())
        self.assertTrue(protected_png.exists())
        self.assertIn(paths[0], removed)
        self.assertNotIn(protected_png, removed)

    def test_existing_unknown_outputs_are_not_overwritten_or_treated_complete(self):
        path = self.csv_path()
        self.write_rows(path, [{"elapsed_s": "0.0", "motor1_permille": "1"}])
        path.with_suffix(".png").symlink_to(path)

        generated, failed = report.generate_missing_for([path])

        self.assertEqual(generated, [])
        self.assertEqual(failed[0]["csv"], path.name)
        self.assertIn("refusing to overwrite", failed[0]["error"])
        self.assertTrue(path.with_suffix(".png").is_symlink())

        regular = self.root / "arm-torque-20261009-120001-000001-1234.csv"
        self.write_rows(regular, [{"elapsed_s": "0.0", "motor1_permille": "1"}])
        regular.with_suffix(".png").write_text("somebody else's chart")

        _, regular_failed = report.generate_missing_for([regular])

        self.assertEqual(regular_failed[0]["csv"], regular.name)
        self.assertEqual(regular.with_suffix(".png").read_text(), "somebody else's chart")

    def test_part_files_recover_only_when_unlocked_and_never_follow_symlinks(self):
        unlocked = self.root / "arm-torque-20261009-120000-000001-1234.csv.part"
        locked = self.root / "arm-torque-20261009-120001-000001-1234.csv.part"
        linked = self.root / "arm-torque-20261009-120002-000001-1234.csv.part"
        self.write_rows(unlocked, [{"elapsed_s": "0.0"}])
        self.write_rows(locked, [{"elapsed_s": "0.0"}])
        target = self.root / "outside.csv.part"
        target.write_text("not a report")
        linked.symlink_to(target)

        with locked.open("r+") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            recovered = report.recover_interrupted_csvs(self.root)

        self.assertEqual(recovered, [unlocked.with_suffix("")])
        self.assertTrue(unlocked.with_suffix("").is_file())
        self.assertTrue(locked.exists())
        self.assertTrue(linked.is_symlink())

    def test_part_recovery_does_not_overwrite_completed_csv(self):
        part = self.root / "arm-torque-20261009-120000-000001-1234.csv.part"
        completed = part.with_suffix("")
        part.write_text("part")
        completed.write_text("completed")

        recovered = report.recover_interrupted_csvs(self.root)

        self.assertEqual(recovered, [])
        self.assertEqual(completed.read_text(), "completed")
        self.assertTrue(part.exists())

    def test_report_failure_keeps_latest_csv_but_still_prunes_older_sessions(self):
        paths = []
        for second in range(4):
            path = self.root / f"arm-torque-20261009-12000{second}-000001-1234.csv"
            self.write_rows(path, [{"elapsed_s": "0.0", "motor1_permille": "1"}])
            paths.append(path)

        latest = paths[-1]

        def fail_latest(path):
            if path == latest:
                raise RuntimeError("plot failed")
            return path.with_suffix(".png")

        with mock.patch.object(report, "generate_report", side_effect=fail_latest):
            result = report.run(self.root, keep=3)

        self.assertTrue(latest.exists())
        self.assertFalse(paths[0].exists())
        self.assertEqual(result["failed"], [{"csv": latest.name, "error": "plot failed"}])
        self.assertEqual(result["kept_csv"], [path.name for path in reversed(paths[1:])])

    def test_run_cleans_old_sessions_before_generating_missing_recent_reports(self):
        paths = []
        for second in range(5):
            path = self.root / f"arm-torque-20261009-12000{second}-000001-1234.csv"
            self.write_rows(path, [{"elapsed_s": "0.0", "motor1_permille": "1"}])
            paths.append(path)
        generated_inputs = []

        def remember(path):
            generated_inputs.append(path)
            return path.with_suffix(".png")

        with mock.patch.object(report, "generate_report", side_effect=remember):
            result = report.run(self.root, keep=3)

        self.assertEqual(generated_inputs, [paths[2], paths[3], paths[4]])
        self.assertFalse(paths[0].exists())
        self.assertFalse(paths[1].exists())
        self.assertEqual(result["kept_csv"], [path.name for path in reversed(paths[2:])])


if __name__ == "__main__":
    unittest.main()
