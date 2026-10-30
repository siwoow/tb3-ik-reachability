"""Safety regressions without ROS networking or real hardware."""

import math
from pathlib import Path
import unittest

from tb3_grasp_ros2.safety import (
    Budget, Limits, SafetyError, Step, check_scan, choose_step,
    permission_gate, step_command,
)
from tb3_grasp_ros2.planner_adapter import (
    load_model, native_query, screen_native_path, tight_floor_clearance,
)

ROOT = Path(__file__).resolve().parents[1]


class SafetyTests(unittest.TestCase):
    """Require fail-closed limits and preserve existing safety thresholds."""

    def test_fixed_caps(self):
        for options in ({'step_m': .051}, {'linear_mps': .026},
                        {'max_steps': 21}, {'max_distance_m': .51},
                        {'step_yaw_rad': math.radians(11)}):
            with self.assertRaises(ValueError):
                Limits(**options).validate()

    def test_segment_and_stop(self):
        limits = Limits()
        step = choose_step((0, 0, 0), (.30, 0, 0), limits, .30)
        self.assertEqual(step.kind, 'translate')
        self.assertLessEqual(step.amount, .05)
        self.assertEqual(step_command(step, (0, 0, 0), (.044, 0, 0), limits), (0, 0, True))
        with self.assertRaises(SafetyError):
            step_command(step, (0, 0, 0), (.051, 0, 0), limits)

    def test_rotation_segment_and_no_reverse(self):
        step = choose_step((0, 0, 0), (-1, 0, 0), Limits(), .3)
        self.assertEqual(step.kind, 'rotate')
        self.assertLessEqual(abs(step.amount), math.radians(10))
        with self.assertRaises(SafetyError):
            step_command(step, (0, 0, 0), (.02, 0, 0), Limits())

    def test_accumulated_distance_and_count(self):
        budget = Budget(Limits(max_steps=1, max_distance_m=.05))
        for x in (0, .02, .04):
            budget.observe((x, 0, 0))
        budget.start_step(Step('translate', .01, 0))
        with self.assertRaises(SafetyError):
            budget.start_step(Step('translate', .01, 0))
        with self.assertRaises(SafetyError):
            budget.observe((.06, 0, 0))

    def test_odometry_reset(self):
        budget = Budget(Limits())
        budget.observe((0, 0, 0))
        with self.assertRaises(SafetyError):
            budget.observe((1, 0, 0))

    def scan(self, ranges, turning=False, translation=(0, 0, 0)):
        check_scan(ranges, -math.pi, 2*math.pi/360, .05, 10,
                   [[1, 0, 0], [0, 1, 0], [0, 0, 1]], translation, turning)

    def test_scan_thresholds_and_missing_coverage(self):
        self.scan([1.]*360)
        bad = [1.]*360
        bad[180] = .44
        with self.assertRaises(SafetyError):
            self.scan(bad)
        bad = [1.]*360
        bad[0] = .29
        with self.assertRaises(SafetyError):
            self.scan(bad, True)
        with self.assertRaises(SafetyError):
            self.scan([math.inf]*360)
        with self.assertRaises(SafetyError):
            self.scan([math.nan]*360)

    def test_real_lidar_translation(self):
        ranges = [1.]*360
        ranges[180] = .5
        with self.assertRaises(SafetyError):
            self.scan(ranges, translation=(-.1, 0, 0))

    def test_separate_explicit_permissions(self):
        self.assertEqual(permission_gate(False, False, False), 'dry_run_plan')
        self.assertEqual(permission_gate(False, False, True), 'reachable_stop')
        self.assertEqual(permission_gate(True, False, True), 'reachable_stop')
        self.assertEqual(permission_gate(True, True, True), 'grasp')
        self.assertEqual(permission_gate(False, True, False), 'dry_run_plan')

    def test_native_ik_and_full_collision_rejections(self):
        model = load_model(str(ROOT.parent/'tb3_grasp_analysis'))
        start = [0, -1, .7, .3]
        blocked = native_query(ROOT/'reference/native_probe', [.20, 0, -.05], -.101, start, .006)
        self.assertTrue(blocked['feasible'])
        self.assertFalse(screen_native_path(model, blocked, start, .019))
        valid = native_query(ROOT/'reference/native_probe', [.30, 0, -.05], -.101, start, .006)
        self.assertTrue(valid['feasible'])
        self.assertTrue(screen_native_path(model, valid, start, .019))
        self.assertGreaterEqual(float(tight_floor_clearance(model, [valid['descent'][-1]], .019).min()), .006)


if __name__ == '__main__':
    unittest.main()
