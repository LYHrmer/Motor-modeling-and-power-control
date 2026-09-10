"""Synthetic recovery and input-boundary tests, not hardware validation."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "fit_power_model.py"
SPEC = importlib.util.spec_from_file_location("fit_power_model", SCRIPT)
FIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIT)


class FittingTests(unittest.TestCase):
    def samples(self, model):
        return [(rpm, command, FIT.predict(model, rpm, command))
                for rpm in (-3000, -1000, 0, 200, 1700, 4000)
                for command in (-12000, -1700, 0, 3200, 9000)]

    def test_exact_recovery_for_all_speed_models(self):
        for speed_model in ("linear", "quadratic", "combined"):
            with self.subTest(speed_model=speed_model):
                expected = {"k_mech": 2e-6, "k_speed_abs": 0.002 if speed_model != "quadratic" else 0.0,
                            "k_speed_sq": 1e-7 if speed_model != "linear" else 0.0,
                            "k_current_sq": 1.2e-7, "p_idle": 2.1, "current_limit": 16000.0}
                samples = self.samples(expected)
                model = FIT.fit(samples, expected["k_mech"], speed_model, 16000)
                for key in expected:
                    self.assertAlmostEqual(model[key], expected[key], delta=max(1e-12, abs(expected[key]) * 1e-6))
                self.assertLess(FIT.metrics(samples, model)["rmse_w"], 1e-8)

    def test_negative_loss_is_projected_to_boundary(self):
        samples = [(rpm, command, -2.0) for rpm in (0, 10, 20, 40) for command in (0, 3, 7, 12)]
        model = FIT.fit(samples, 0, "quadratic", 16000)
        self.assertTrue(all(value >= 0 for value in model.values()))
        self.assertEqual(model["p_idle"], 0)
        self.assertEqual(model["k_current_sq"], 0)
        self.assertTrue(FIT.fit_warnings(model))

    def test_unidentifiable_data_rejected(self):
        with self.assertRaisesRegex(ValueError, "excitation"):
            FIT.fit([(100, 200, 3)] * 20, 0, "quadratic", 16000)

    def test_legacy_alignment_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.csv"
            path.write_text(
                "timestamp_current;current;timestamp_give_current;give_current;timestamp_motor_chassis[0].speed_rpm;motor_chassis[0].speed_rpm\n"
                "100;1;101;500;102;200\n"
                "100;2;110;600;101;300\n"
                "100;nan;101;500;102;200\n", encoding="ascii")
            rows, records = FIT.read_samples([path], "legacy", 24.0, 5)
            self.assertEqual(rows, [(200.0, 500.0, 24.0)])
            self.assertEqual(records[0]["misaligned_rows"], 1)
            self.assertEqual(records[0]["invalid_rows"], 1)

    def test_cli_holdout_and_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = {"k_mech": 2e-6, "k_speed_abs": 0, "k_speed_sq": 1e-7,
                     "k_current_sq": 1.2e-7, "p_idle": 2.1, "current_limit": 16000}
            rows = self.samples(model)
            for name, subset in (("train.csv", rows[::2]), ("valid.csv", rows[1::2])):
                (root / name).write_text("rpm,current_command,power_w\n" + "".join(
                    f"{rpm},{command},{power}\n" for rpm, command, power in subset), encoding="ascii")
            result = subprocess.run([sys.executable, str(SCRIPT), str(root / "train.csv"),
                                     "--validation", str(root / "valid.csv"), "--k-mech", "2e-6",
                                     "--current-limit", "16000", "--output", str(root / "model.json"),
                                     "--header", str(root / "model.h")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads((root / "model.json").read_text())
            self.assertFalse(report["hardware_validated"])
            self.assertLess(report["validation"]["rmse_w"], 1e-8)
            self.assertIn("lp_fitted_motor_model", (root / "model.h").read_text())

    def test_cli_rejects_train_validation_overlap(self):
        result = subprocess.run([sys.executable, str(SCRIPT), "same.csv", "--validation", "same.csv",
                                 "--k-mech", "2e-6", "--current-limit", "16000", "--output", "unused.json"],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("different files", result.stderr)


if __name__ == "__main__":
    unittest.main()
