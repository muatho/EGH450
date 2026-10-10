#!/usr/bin/env python3
import unittest

from depthai_publisher.mission_announcer_core import (
    mission_event_announcement,
    payload_announcement,
)


class MissionAnnouncementTest(unittest.TestCase):
    def test_mission_start(self):
        self.assertEqual(
            mission_event_announcement("/guidance", "Starting waypoint mission"),
            ("mission_start", "Starting the mission. Ready to go."),
        )

    def test_landing_marker_selection(self):
        self.assertEqual(
            mission_event_announcement(
                "/guidance", "[LAND] Flying to ArUco 7 at (1.20, -2.30)"
            ),
            ("landing_marker:7", "Landing sequence started for ArUco marker 7."),
        )

    def test_landing_complete(self):
        self.assertEqual(
            mission_event_announcement("/guidance", "[LAND] Landing complete!"),
            ("mission_complete", "Landing complete. Mission complete."),
        )

    def test_ignores_unrelated_logs(self):
        self.assertIsNone(
            mission_event_announcement("/mavros", "[LAND] Landing complete!")
        )
        self.assertIsNone(
            mission_event_announcement("/guidance", "[LAND] Landing failed")
        )

    def test_payload_announcements(self):
        self.assertEqual(
            payload_announcement("a"),
            ("payload:a:person", "Payload A deployed for person."),
        )
        self.assertEqual(
            payload_announcement("B"),
            ("payload:b:backpack", "Payload B deployed for backpack."),
        )
        self.assertIsNone(payload_announcement("unknown"))


if __name__ == "__main__":
    unittest.main()
