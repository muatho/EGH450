"""
Flight-control boundary for the autonomous mission.

This module owns the SPAR action client and exposes flight-controller
availability to the mission state machine. Mission logic does not construct
SPAR action goals or access the ROS action client directly.

Constructing FlightInterface is non-blocking and sends no aircraft commands.
"""

import actionlib
import rospy
import math

from enum import Enum, auto
from actionlib_msgs.msg import GoalStatus

from spar_msgs.msg import FlightMotionAction, FlightMotionGoal
from spar_node.mission_types import Waypoint


class FlightGoalState(Enum):
    """Mission-facing status of the current SPAR flight command."""

    IDLE = auto()
    PENDING = auto()
    ACTIVE = auto()
    SUCCEEDED = auto()
    CANCELLED = auto()
    FAILED = auto()


class FlightInterface:
    """
    Provide controlled access to the SPAR flight-action server.

    Connection readiness is checked explicitly during PREFLIGHT so the
    mission controller remains responsible for startup and failure decisions.
    """

    def __init__(self):
        # Retain the established `spar/flight` interface while allowing the
        # action topic to be overridden through a ROS parameter.
        self._action_topic = rospy.get_param(
            "~action_topic",
            "spar/flight",
        )

        # Creating the client prepares the ROS connection but does not arm,
        # move or otherwise command the aircraft.
        self._client = actionlib.SimpleActionClient(
            self._action_topic,
            FlightMotionAction,
        )

        self._connected = False

        # Goal state is only meaningful after this interface has submitted a
        # command to SPAR.
        self._goal_submitted = False

        # SPAR motion parameters and defaults
        self._linear_velocity = rospy.get_param(
            "~vel_linear",
            0.4,
        )

        self._yaw_velocity = rospy.get_param(
            "~vel_yaw",
            0.2,
        )

        self._position_tolerance = rospy.get_param(
            "~acc_pos",
            0.3,
        )

        self._yaw_tolerance = rospy.get_param(
            "~acc_yaw",
            0.2,
        )

        # Preserve the established controlled-descent speed used by the legacy
        # landing and abort-landing paths.
        self._landing_velocity = rospy.get_param(
            "~landing_speed",
            0.2,
        )

        motion_parameters = {
            "~vel_linear": self._linear_velocity,
            "~vel_yaw": self._yaw_velocity,
            "~acc_pos": self._position_tolerance,
            "~acc_yaw": self._yaw_tolerance,
            "~landing_speed": self._landing_velocity,
        }

        for parameter_name, value in motion_parameters.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(
                        parameter_name
                    )
                )

        rospy.loginfo(
            "Flight interface created for SPAR action topic: %s",
            self._action_topic,
        )

    @property
    def is_connected(self) -> bool:
        """Return whether SPAR passed the latest readiness check."""
        return self._connected

    def wait_until_ready(self, timeout_seconds: float) -> bool:
        """
        Wait for SPAR to become available within a finite timeout.

        Args:
            timeout_seconds: Maximum number of seconds to wait. It must be
                greater than zero.

        Returns:
            True when the SPAR action server is available.
            False when the timeout expires first.
        """

        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be greater than zero")

        rospy.loginfo(
            "Waiting up to %.1f seconds for SPAR on %s",
            timeout_seconds,
            self._action_topic,
        )

        self._connected = self._client.wait_for_server(
            rospy.Duration(timeout_seconds)
        )

        if self._connected:
            rospy.loginfo("SPAR flight-action server is available")
        else:
            rospy.logerr(
                "SPAR flight-action server was unavailable after %.1f seconds",
                timeout_seconds,
            )

        return self._connected

    def send_takeoff(self, height: float) -> bool:
        """
        Command a vertical take-off to the requested height.

        MissionController selects the take-off height and decides how command
        success or failure affects the mission. FlightInterface only validates
        the command and translates it into the established SPAR action format.

        Args:
            height: Desired take-off height in metres.

        Returns:
            True when the goal was submitted.
            False when SPAR is not connected.
        """

        if not math.isfinite(height) or height <= 0:
            raise ValueError(
                "Take-off height must be finite and greater than zero"
            )

        if not self._connected:
            rospy.logerr(
                "Cannot send take-off command because SPAR is not connected"
            )
            return False

        goal = FlightMotionGoal()

        # Use SPAR's dedicated take-off motion rather than treating the first
        # search waypoint as an implicit take-off command.
        goal.motion = FlightMotionGoal.MOTION_TAKEOFF
        goal.position.z = height

        # SPAR ignores horizontal velocity and horizontal position during
        # MOTION_TAKEOFF. The established linear velocity is used for ascent.
        goal.velocity_vertical = self._linear_velocity

        goal.wait_for_convergence = True
        goal.position_radius = self._position_tolerance
        goal.yaw_range = self._yaw_tolerance

        rospy.loginfo(
            "Sending SPAR take-off command to %.2f m",
            height,
        )

        self._client.send_goal(goal)
        self._goal_submitted = True
        return True

    def send_land(self) -> bool:
        """
        Command the aircraft to land vertically at its current location.

        Landing remains available when battery telemetry is unsafe because a
        low-battery condition must not prevent the controller from requesting
        a controlled descent.

        Returns:
            True when the landing goal was submitted.
            False when SPAR is not connected.
        """

        if not self._connected:
            rospy.logerr(
                "Cannot send landing command because SPAR is not connected"
            )
            return False

        goal = FlightMotionGoal()
        goal.motion = FlightMotionGoal.MOTION_LAND
        goal.velocity_vertical = self._landing_velocity

        rospy.logwarn(
            "Sending SPAR landing command at %.2f m/s",
            self._landing_velocity,
        )

        self._client.send_goal(goal)
        self._goal_submitted = True
        return True

    def send_waypoint(self, waypoint: Waypoint) -> bool:
        """
        Send one map-frame waypoint to the SPAR flight controller.

        SPAR reports completion only after the aircraft converges within the
        configured position and yaw tolerances. Goal completion is observed
        separately so this method does not block ROS callbacks.

        Args:
            waypoint: Validated destination position and yaw.

        Returns:
            True when the goal was submitted.
            False when SPAR is not connected.
        """

        if not isinstance(waypoint, Waypoint):
            raise TypeError("waypoint must be a Waypoint")

        if not self._connected:
            rospy.logerr(
                "Cannot send waypoint because SPAR is not connected"
            )
            return False

        goal = FlightMotionGoal()

        # Preserve the established SPAR waypoint command and convergence
        # settings used by test_breadcrumb_camera.send_wp().
        goal.motion = FlightMotionGoal.MOTION_GOTO

        goal.position.x = waypoint.x
        goal.position.y = waypoint.y
        goal.position.z = waypoint.z
        goal.yaw = waypoint.yaw

        goal.velocity_vertical = self._linear_velocity
        goal.velocity_horizontal = self._linear_velocity
        goal.yawrate = self._yaw_velocity

        goal.wait_for_convergence = True
        goal.position_radius = self._position_tolerance
        goal.yaw_range = self._yaw_tolerance

        rospy.loginfo(
            "Sending SPAR waypoint: "
            "[x=%.2f, y=%.2f, z=%.2f, yaw=%.2f]",
            waypoint.x,
            waypoint.y,
            waypoint.z,
            waypoint.yaw,
        )

        self._client.send_goal(goal)
        self._goal_submitted = True
        return True

    def goal_state(self) -> FlightGoalState:
        """
        Return the mission-facing state of the current SPAR goal.

        ROS action states are translated here so MissionController does not
        depend directly on actionlib status constants.
        """

        if not self._goal_submitted:
            return FlightGoalState.IDLE

        ros_state = self._client.get_state()

        if ros_state in (
            GoalStatus.PENDING,
            GoalStatus.RECALLING,
        ):
            return FlightGoalState.PENDING

        if ros_state in (
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
        ):
            return FlightGoalState.ACTIVE

        if ros_state == GoalStatus.SUCCEEDED:
            return FlightGoalState.SUCCEEDED

        if ros_state in (
            GoalStatus.PREEMPTED,
            GoalStatus.RECALLED,
        ):
            return FlightGoalState.CANCELLED

        if ros_state in (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.LOST,
        ):
            return FlightGoalState.FAILED

        rospy.logerr_throttle(
            2.0,
            "SPAR returned unknown action state: %s",
            ros_state,
        )
        return FlightGoalState.FAILED

    def cancel_current_goal(self) -> bool:
        """
        Request cancellation of the current SPAR goal.

        Returns:
            True when a submitted goal existed and cancellation was requested.
            False when there was no goal to cancel.
        """

        if not self._goal_submitted:
            return False

        rospy.logwarn("Cancelling current SPAR flight goal")
        self._client.cancel_goal()
        return True