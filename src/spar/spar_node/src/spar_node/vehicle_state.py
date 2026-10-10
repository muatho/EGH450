"""
Vehicle pose and battery state for the autonomous mission.

This module normalises the different real-aircraft and emulator pose messages
into one internal representation. It also records battery telemetry and the
arrival time of safety-critical data.

The class observes the vehicle only. It does not make mission-state decisions
or send flight commands.
"""

import math
import threading
import time

import rospy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from typing import Optional

class VehicleState:
    """Maintain the latest validated vehicle pose and battery information."""

    def __init__(self):
        self._lock = threading.RLock()
        self._update_event = threading.Event()

        # External-position freshness is tracked separately from PX4's local
        # position estimate. On the real aircraft this directly observes the
        # OptiTrack feed entering MAVROS.
        self._external_position_received_at = None

        # Real flight and the emulator publish different pose message types.
        # Both are converted into the same Point-based internal state.
        self._pose_source = rospy.get_param(
            "~pose_source",
            "real",
        )

        if self._pose_source not in ("real", "emulator"):
            raise ValueError("~pose_source must be 'real' or 'emulator'")

        default_pose_topic = (
            "/uavasr/odom"
            if self._pose_source == "emulator"
            else "/mavros/local_position/pose"
        )

        default_battery_topic = (
            "/uavasr/battery"
            if self._pose_source == "emulator"
            else "/mavros/battery"
        )

        self._optitrack_topic = rospy.get_param(
            "~optitrack_topic",
            "/mavros/vision_pose/pose",
        )

        self._pose_topic = rospy.get_param(
            "~pose_topic",
            default_pose_topic,
        )

        self._battery_topic = rospy.get_param(
            "~battery_topic",
            default_battery_topic,
        )

        # Pose data must remain much fresher than slowly changing battery
        # telemetry. These defaults retain the established mission behaviour.
        self._pose_timeout = rospy.get_param(
            "~pose_timeout",
            1.0,
        )

        self._battery_timeout = rospy.get_param(
            "~battery_timeout",
            5.0,
        )

        self._minimum_battery = rospy.get_param(
            "~minimum_battery",
            0.20,
        )

        if self._pose_timeout <= 0:
            raise ValueError("~pose_timeout must be greater than zero")

        if self._battery_timeout <= 0:
            raise ValueError("~battery_timeout must be greater than zero")

        if not 0.0 <= self._minimum_battery <= 1.0:
            raise ValueError("~minimum_battery must be between zero and one")

        self._position = None
        self._pose_stamp = None
        self._pose_received_at = None

        self._battery_percentage = None
        self._battery_received_at = None

        if self._pose_source == "emulator":
            self._pose_subscriber = rospy.Subscriber(
                self._pose_topic,
                Odometry,
                self._odometry_callback,
                queue_size=1,
            )

            # The emulator has no separate OptiTrack topic. Its odometry
            # callback provides the equivalent external-position heartbeat.
            self._optitrack_subscriber = None

        else:
            self._pose_subscriber = rospy.Subscriber(
                self._pose_topic,
                PoseStamped,
                self._pose_callback,
                queue_size=1,
            )
            self._optitrack_subscriber = rospy.Subscriber(
                self._optitrack_topic,
                PoseStamped,
                self._optitrack_callback,
                queue_size=1,
            )

        self._battery_subscriber = rospy.Subscriber(
            self._battery_topic,
            BatteryState,
            self._battery_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Vehicle state using pose topic %s and battery topic %s",
            self._pose_topic,
            self._battery_topic,
        )

    def _pose_callback(self, message: PoseStamped):
        """Receive a real-aircraft PoseStamped message."""
        self._accept_pose(
            message.pose.position,
            message.header.stamp,
        )

    def _odometry_callback(self, message: Odometry):
        """
        Receive emulator odometry.

        Emulator odometry represents both the usable vehicle pose and the
        external-position source used to exercise freshness supervision.
        """

        accepted = self._accept_pose(
            message.pose.pose.position,
            message.header.stamp,
        )

        if accepted:
            with self._lock:
                self._external_position_received_at = time.monotonic()
                self._update_event.set()

    def _optitrack_callback(self, message: PoseStamped):
        """
        Record receipt of a valid OptiTrack pose entering MAVROS.

        This callback tracks source availability only. Vehicle navigation
        continues to use PX4's fused local-position estimate.
        """

        position = message.pose.position
        values = (
            position.x,
            position.y,
            position.z,
        )

        message_age = (
            rospy.Time.now() - message.header.stamp
        ).to_sec()

        if (
            not all(math.isfinite(value) for value in values)
            or not -0.5 <= message_age <= self._pose_timeout
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring invalid or stale OptiTrack pose",
            )
            return

        with self._lock:
            self._external_position_received_at = time.monotonic()
            self._update_event.set()

    def _accept_pose(self, position: Point, stamp):
        """Validate and store a position from either supported pose source."""

        values = (
            position.x,
            position.y,
            position.z,
        )

        message_age = (rospy.Time.now() - stamp).to_sec()

        if (
            not all(math.isfinite(value) for value in values)
            or not -0.5 <= message_age <= self._pose_timeout
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring invalid or stale vehicle pose",
            )
            return False

        with self._lock:
            # Store a copy instead of retaining part of the incoming ROS
            # message, keeping internal state independent of message lifetime.
            self._position = Point(
                x=position.x,
                y=position.y,
                z=position.z,
            )
            self._pose_stamp = stamp
            self._pose_received_at = time.monotonic()
            self._update_event.set()

        return True

    def _battery_callback(self, message: BatteryState):
        """Validate and store battery percentage in the ROS 0-to-1 format."""

        percentage = message.percentage

        if (
            not math.isfinite(percentage)
            or not 0.0 <= percentage <= 1.0
        ):
            rospy.logerr_throttle(
                2.0,
                "Ignoring invalid battery percentage",
            )
            return

        with self._lock:
            self._battery_percentage = percentage
            self._battery_received_at = time.monotonic()
            self._update_event.set()

        if percentage <= self._minimum_battery + 1e-7:
            rospy.logerr_throttle(
                2.0,
                "Battery is at or below the configured minimum: %.1f%%",
                percentage * 100.0,
            )

    def external_position_age(self) -> Optional[float]:
        """
        Return seconds since the latest external-position observation.

        Real flight uses the direct OptiTrack-to-MAVROS feed. Emulator mode
        uses accepted odometry as its equivalent source. None means that no
        valid observation has arrived.
        """

        with self._lock:
            if self._external_position_received_at is None:
                return None

            return (
                time.monotonic()
                - self._external_position_received_at
            )

    def external_position_is_ready(self) -> bool:
        """
        Return whether the external-position source is currently fresh.

        This is a strict startup requirement. The longer in-flight loss grace
        period is applied separately by MissionController.
        """

        age = self.external_position_age()

        return (
            age is not None
            and age <= self._pose_timeout
        )

    def current_position(self) -> Optional[Point]:
        """
        Return a copy of the latest usable map-frame position.

        Returning a copy prevents mission code from modifying VehicleState's
        stored observation. None is returned when no valid fresh pose exists.
        """

        with self._lock:
            if (
                self._pose_received_at is None
                or self._pose_stamp is None
                or self._position is None
            ):
                return None

            reception_age = (
                time.monotonic() - self._pose_received_at
            )
            message_age = (
                rospy.Time.now() - self._pose_stamp
            ).to_sec()

            if (
                reception_age > self._pose_timeout
                or not -0.5 <= message_age <= self._pose_timeout
            ):
                return None

            return Point(
                x=self._position.x,
                y=self._position.y,
                z=self._position.z,
            )

    def pose_is_ready(self) -> bool:
        """Return whether a usable current position is available."""

        return self.current_position() is not None

    def battery_is_ready(self) -> bool:
        """Return whether battery telemetry is fresh and above the minimum."""

        with self._lock:
            if (
                self._battery_received_at is None
                or self._battery_percentage is None
            ):
                return False

            reception_age = (
                time.monotonic() - self._battery_received_at
            )

            return (
                reception_age <= self._battery_timeout
                and self._battery_percentage
                > self._minimum_battery + 1e-7
            )

    def wait_until_ready(self, timeout_seconds: float) -> bool:
        """
        Wait for fresh pose, external positioning and safe battery telemetry.

        This readiness check applies during PREFLIGHT. It does not implement
        the separate ten-second external-position loss policy used in flight.

        Args:
            timeout_seconds: Maximum number of seconds to wait. It must be
                greater than zero.

        Returns:
            True when every required telemetry source is ready.
            False when the timeout expires or ROS shuts down.
        """

        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be greater than zero")

        rospy.loginfo(
            "Waiting up to %.1f seconds for vehicle telemetry",
            timeout_seconds,
        )

        deadline = time.monotonic() + timeout_seconds

        while not rospy.is_shutdown():
            if (
                self.pose_is_ready()
                and self.external_position_is_ready()
                and self.battery_is_ready()
            ):
                rospy.loginfo(
                    "Vehicle pose, external positioning and "
                    "battery telemetry are ready"
                )
                return True

            with self._lock:
                battery_is_unsafe = (
                    self._battery_percentage is not None
                    and self._battery_percentage
                    <= self._minimum_battery + 1e-7
                )

            # A confirmed low battery cannot become acceptable merely by
            # waiting for the remainder of the startup timeout.
            if battery_is_unsafe:
                rospy.logerr(
                    "Vehicle battery is below the startup requirement"
                )
                return False

            remaining = deadline - time.monotonic()

            if remaining <= 0:
                break

            self._update_event.wait(
                min(0.1, remaining)
            )
            self._update_event.clear()

        if not self.pose_is_ready():
            rospy.logerr(
                "Vehicle pose was unavailable or stale during preflight"
            )

        if not self.external_position_is_ready():
            rospy.logerr(
                "External-position telemetry was unavailable or stale "
                "during preflight"
            )

        if not self.battery_is_ready():
            rospy.logerr(
                "Battery telemetry was unavailable, stale or unsafe "
                "during preflight"
            )

        return False
