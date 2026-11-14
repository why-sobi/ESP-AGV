import unittest

import numpy as np

from evaluate_faults import evaluate_run, primary_model, severity_axis


def zero_reconstruct(x):
    return np.zeros_like(x)


class EvaluateFaultsTests(unittest.TestCase):
    def test_fault_types_map_to_expected_model(self):
        self.assertEqual(primary_model("motor_gain"), "motor")
        self.assertEqual(primary_model("encoder_stuck"), "motor")
        self.assertEqual(primary_model("ir_drift"), "ir")

    def test_severity_axis_grows_with_fault_strength(self):
        mild, _ = severity_axis("motor_gain", 0.9)
        strong, label = severity_axis("motor_gain", 0.5)
        self.assertLess(mild, strong)
        self.assertAlmostEqual(strong, 50.0)
        self.assertIn("power loss", label)
        self.assertEqual(severity_axis("ir_noise", 75.0)[0], 75.0)

    def test_detection_delay_and_pre_onset_alarm(self):
        window_size, count = 2, 20
        starts = np.arange(count) * 0.1
        ends = starts + 0.1
        # Score = mean of squares; windows become large from index 12 on.
        motor = np.zeros((count, window_size, 1), dtype=np.float32)
        motor[12:] = 10.0
        ir = np.zeros((count, window_size, 1), dtype=np.float32)
        ir[2:5] = 10.0  # a pre-onset burst inside the warm-up, then after it
        detectors = {
            name: (zero_reconstruct, 1.0, np.zeros(1), np.ones(1)) for name in ("motor", "ir")
        }

        results = evaluate_run({"motor": motor, "ir": ir}, starts, ends, onset=1.15,
                               detectors=detectors, k=2, n=3, warmup_s=0.25)

        # Motor: windows 12 and 13 exceed -> alarm at window 13 (end 1.4 s).
        self.assertTrue(results["motor"]["detected"])
        self.assertAlmostEqual(results["motor"]["delay_s"], 1.4 - 1.15)
        self.assertFalse(results["motor"]["pre_onset_false_alarm"])
        # IR: windows 3-4 are monitored (start >= 0.25) and exceed -> pre-onset alarm.
        self.assertTrue(results["ir"]["pre_onset_false_alarm"])
        self.assertFalse(results["ir"]["detected"])


if __name__ == "__main__":
    unittest.main()
