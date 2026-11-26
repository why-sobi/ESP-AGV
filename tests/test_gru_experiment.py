import unittest

import numpy as np

from scripts.models.gru_experiment import (
    alarms,
    no_alarm_threshold,
    stuck_score,
    trailing_mean,
    window_mask,
)


class GruExperimentTests(unittest.TestCase):
    def test_no_alarm_threshold_sits_above_kth_largest_in_any_span(self):
        scores = np.array([0.1, 5.0, 0.2, 4.0, 3.0, 0.1, 0.1])
        mask = np.ones(len(scores), bool)
        threshold = no_alarm_threshold([(scores, mask)], k=2, n=3)
        # Span [5.0, 0.2, 4.0] has 4.0 as its 2nd largest.
        self.assertGreater(threshold, 4.0)
        self.assertLess(threshold, 4.0001)
        self.assertFalse(alarms(scores, mask, threshold, 2, 3).any())
        self.assertTrue(alarms(scores, mask, 3.5, 2, 3).any())

    def test_unmonitored_samples_do_not_set_threshold_or_alarm(self):
        scores = np.array([9.0, 9.0, 9.0, 0.1, 0.1])
        mask = np.array([False, False, False, True, True])
        threshold = no_alarm_threshold([(scores, mask)], k=2, n=3)
        self.assertLess(threshold, 0.2)
        self.assertFalse(alarms(scores, mask, threshold, 2, 3).any())

    def test_trailing_mean_is_nan_before_a_full_window(self):
        values = np.arange(5, dtype=float)[:, None]
        out = trailing_mean(values, 3)
        self.assertTrue(np.isnan(out[:2]).all())
        np.testing.assert_allclose(out[2:, 0], [1.0, 2.0, 3.0])

    def test_window_mask_needs_warmup_and_no_manoeuvre(self):
        run = {"times": np.arange(10) * 0.1,
               "unmonitored": np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0], bool)}
        mask = window_mask(run, window=3, warmup_s=0.15)
        # Window ending at t covers t-2..t; starts need time >= 0.15 (index 2+).
        expected = [False, False, False, False, True, True, False, False, False, True]
        self.assertEqual(mask.tolist(), expected)

    def test_stuck_sensor_flagged_only_while_driving(self):
        rng = np.random.default_rng(0)
        count = 80
        driving = np.tile([2.4, 2.4, 2.4, 2.4], (count, 1))
        ir = 400 + 100 * np.sin(np.arange(count)[:, None] * 0.3 + np.arange(3)) \
            + rng.normal(0, 2, (count, 3))
        ir[30:, 1] = 300.0  # center sensor stuck from sample 30
        run = {"raw_ir": ir, "raw_motor": driving}
        score = stuck_score(run, samples=10, eps=0.5, speed=1.0)
        self.assertEqual(score[:40].sum(), 0)  # needs 10 flat differences
        self.assertTrue((score[40:] == 1).all())

        # Off the track the healthy sensors read flat floor (noise only): the
        # stuck one is still caught while the robot drives.
        off_track = 765 + rng.normal(0, 2, (count, 3))
        off_track[:, 1] = 300.0
        self.assertTrue(stuck_score({"raw_ir": off_track, "raw_motor": driving},
                                    10, 0.5, 1.0)[10:].all())

        stopped = {"raw_ir": np.full((count, 3), 300.0), "raw_motor": np.zeros((count, 4))}
        self.assertEqual(stuck_score(stopped, 10, 0.5, 1.0).sum(), 0)


if __name__ == "__main__":
    unittest.main()
