#!/usr/bin/env python3
import unittest

from depthai_publisher.object_announcer_core import (
    TargetAnnouncementGate,
    announcement_text,
)


class TargetAnnouncementGateTest(unittest.TestCase):
    def test_requires_consecutive_nearby_observations(self):
        gate = TargetAnnouncementGate(3, 1.0)
        self.assertEqual(gate.update([("person", (1, 2, 0))]), [])
        self.assertEqual(gate.update([]), [])
        self.assertEqual(gate.update([("person", (1, 2, 0))]), [])
        self.assertEqual(gate.update([("person", (1.2, 2, 0))]), [])
        self.assertEqual(len(gate.update([("person", (1.1, 2.1, 0))])), 1)

    def test_does_not_repeat_same_physical_target(self):
        gate = TargetAnnouncementGate(1, 1.0)
        self.assertEqual(len(gate.update([("backpack", (2, 3, 0))], now=0)), 1)
        self.assertEqual(gate.update([("backpack", (2.4, 3, 0))], now=30), [])

    def test_reannounces_same_target_after_cooldown(self):
        gate = TargetAnnouncementGate(1, 1.0, cooldown_seconds=60)
        self.assertEqual(len(gate.update([("backpack", (2, 3, 0))], now=0)), 1)
        self.assertEqual(gate.update([("backpack", (2.2, 3, 0))], now=59), [])
        self.assertEqual(len(gate.update([("backpack", (2.2, 3, 0))], now=60)), 1)

    def test_reports_separate_same_type_targets(self):
        gate = TargetAnnouncementGate(1, 1.0)
        self.assertEqual(len(gate.update([("person", (0, 0, 0)), ("person", (4, 5, 0))])), 2)

    def test_marker_id_is_not_repeated_when_position_jumps(self):
        gate = TargetAnnouncementGate(1, 1.0, deduplicate_by_type=True,
                                      cooldown_seconds=60)
        self.assertEqual(len(gate.update([("ArUco marker 66", (0, 0, 0))], now=0)), 1)
        self.assertEqual(
            gate.update([("ArUco marker 66", (-399.5, -1453.1, 0))], now=30),
            [],
        )
        self.assertEqual(
            len(gate.update([("ArUco marker 71", (-399.6, -1453.4, 0))], now=30)),
            1,
        )
        self.assertEqual(
            len(gate.update([("ArUco marker 66", (500, 500, 0))], now=60)),
            1,
        )

    def test_formats_requirement_phrase(self):
        self.assertEqual(
            announcement_text("person", (1.24, -3.56, 0)),
            "Detected person at X 1.2 metres, Y -3.6 metres.",
        )
        self.assertEqual(
            announcement_text("ArUco marker 7", (2, 8, 0)),
            "Detected ArUco marker 7 at X 2.0 metres, Y 8.0 metres.",
        )

    def test_rejects_invalid_configuration(self):
        with self.assertRaises(ValueError):
            TargetAnnouncementGate(0)
        with self.assertRaises(ValueError):
            TargetAnnouncementGate(match_radius=0)
        with self.assertRaises(ValueError):
            TargetAnnouncementGate(cooldown_seconds=0)


if __name__ == "__main__":
    unittest.main()
