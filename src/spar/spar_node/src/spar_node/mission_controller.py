"""
Top-level autonomous mission state machine.

This module decides which phase of the mission is currently active and which
state transitions are permitted. It will eventually coordinate flight,
target evidence, operator vetoes and payload deployment through separate
interfaces.

It does not calculate obstacle-avoiding paths, run object detection or drive
payload hardware directly. Those responsibilities remain isolated behind
their respective ROS interfaces.

Legacy successor:
    spar_node/scripts/unused/test_breadcrumb_camera
    Primarily the former Guidance class and its mission-control flags.
"""

from enum import Enum, auto
from typing import Optional

import rospy
import threading
import time
import math

from spar_node.flight_interface import FlightGoalState, FlightInterface
from spar_node.breadcrumb_interface import BreadcrumbInterface
from spar_node.vehicle_state import VehicleState
from spar_node.search_plan import build_default_search_plan
from spar_node.mission_types import Waypoint
from spar_node.target_observations import TargetObservationBuffer
from spar_node.target_evidence import EvidenceCandidate, TargetEvidenceAccumulator
from spar_node.operator_interface import OperatorDecision, OperatorInterface
from spar_node.payload_interface import PayloadInterface
from spar_node.landing_marker_interface import LandingMarker, LandingMarkerInterface
from spar_node.test_target_interface import TestTargetInterface
from spar_node.target_marker_publisher import TargetMarkerPublisher

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path


class MissionState(Enum):
    """
    High-level phases of the autonomous mission.

    Only one state may be active at a time. This replaces the collection of
    partially overlapping Boolean flags used by test_breadcrumb_camera.
    """

    # Non-commanding startup state. Required ROS services, topics, pose data
    # and safety information must be checked before take-off is permitted.
    PREFLIGHT = auto()

    # Take-off has been requested and the controller is waiting for the
    # aircraft to reach its initial search altitude.
    TAKEOFF = auto()

    # Follow the planned search route while accumulating target evidence.
    # Seeing one raw detection does not automatically leave this state.
    SEARCH = auto()

    # Temporarily leave the search route and travel toward a confirmed
    # candidate's estimated ground position.
    DIVERT = auto()

    # Adjust aircraft position until the candidate is sufficiently central
    # in the camera image for reliable final assessment.
    CENTRE = auto()

    # Re-evaluate the candidate after centring. Deployment remains prohibited
    # unless the fresh visual evidence and applicable safety gates pass.
    REASSESS = auto()

    # Offer the operator a time-limited veto only when the GCS is confirmed
    # to be receiving fresh imagery. Silence does not prevent autonomy.
    WAIT_FOR_VETO = auto()

    # Issue the class-appropriate payload command and confirm that the mission
    # has recorded that payload as used.
    DEPLOY = auto()

    # Return to an appropriate point in the interrupted search rather than
    # treating the diversion as completion of the full mission.
    RETURN_TO_SEARCH = auto()

    # Hold position after mission searching is complete while the mission
    # pane asks the operator to select one of the observed ArUco markers.
    WAIT_FOR_LANDING_SELECTION = auto()

    # Follow a Breadcrumb route to the selected ArUco at survey altitude,
    # then descend directly above it to the configured approach altitude.
    APPROACH_LANDING = auto()

    # Execute the controlled landing procedure after search completion or
    # when required by a recoverable mission termination.
    LAND = auto()

    # Terminal state for a mission that has ended and requires no more action.
    COMPLETE = auto()

    # Protective state entered after a serious fault or violated safety
    # condition. ABORT decides whether controlled landing is still possible.
    ABORT = auto()


class DeploymentPhase(Enum):
    """Non-blocking phases performed while the mission is in DEPLOY."""

    IDLE = auto()
    DESCENDING = auto()
    WAITING_FOR_PAYLOAD = auto()
    ASCENDING = auto()


class LandingApproachPhase(Enum):
    """Non-blocking phases used to reach the selected landing ArUco."""

    PLANNING = auto()
    ROUTING = auto()
    DESCENDING = auto()


class MissionController:
    # This table describes mission structure, not the detailed evidence or
    # safety conditions that trigger transitions. Those conditions will be
    # implemented separately and must request state changes through
    # transition_to().
    #
    # Keeping permitted transitions in one place prevents callbacks from
    # bypassing required phases. In particular, raw detections cannot jump
    # directly from SEARCH to DEPLOY.

    LEGAL_TRANSITIONS = {
        MissionState.PREFLIGHT: {
            MissionState.TAKEOFF,
            MissionState.ABORT,
        },
        MissionState.TAKEOFF: {
            MissionState.SEARCH,
            MissionState.LAND,
            MissionState.ABORT,
        },
        MissionState.SEARCH: {
            MissionState.DIVERT,
            MissionState.WAIT_FOR_LANDING_SELECTION,
            MissionState.ABORT,
        },
        MissionState.DIVERT: {
            MissionState.CENTRE,
            # Explicitly enabled test targets bypass visual confirmation after
            # reaching their obstacle-free injected map location.
            MissionState.DEPLOY,
            MissionState.RETURN_TO_SEARCH,
            MissionState.ABORT,
        },
        MissionState.CENTRE: {
            MissionState.REASSESS,
            MissionState.RETURN_TO_SEARCH,
            MissionState.ABORT,
        },
        MissionState.REASSESS: {
            MissionState.CENTRE,
            # Fresh GCS imagery exists, so offer the optional timed veto.
            MissionState.WAIT_FOR_VETO,

            # No fresh GCS imagery exists, but all autonomous confirmation
            # and safety conditions pass. Autonomy must not wait indefinitely.
            MissionState.DEPLOY,

            # Confirmation was lost, a veto was applied through another
            # decision path, or the candidate must be reacquired.
            MissionState.RETURN_TO_SEARCH,

            MissionState.ABORT,
        },
        MissionState.WAIT_FOR_VETO: {
            MissionState.CENTRE,
            MissionState.DEPLOY,
            MissionState.RETURN_TO_SEARCH,
            MissionState.ABORT,
        },
        MissionState.DEPLOY: {
            MissionState.RETURN_TO_SEARCH,
            MissionState.WAIT_FOR_LANDING_SELECTION,
            MissionState.ABORT,
        },
        MissionState.RETURN_TO_SEARCH: {
            MissionState.SEARCH,
            MissionState.ABORT,
        },
        MissionState.WAIT_FOR_LANDING_SELECTION: {
            MissionState.APPROACH_LANDING,
            MissionState.ABORT,
        },
        MissionState.APPROACH_LANDING: {
            MissionState.LAND,
            MissionState.ABORT,
        },
        MissionState.LAND: {
            MissionState.COMPLETE,
            MissionState.ABORT,
        },
        MissionState.ABORT: {
            MissionState.LAND,
            MissionState.COMPLETE,
        },
        MissionState.COMPLETE: set(),
    }

    def __init__(
        self,
        flight: FlightInterface,
        breadcrumb: BreadcrumbInterface,
        vehicle: VehicleState,
        target_observations: TargetObservationBuffer,
        target_evidence: TargetEvidenceAccumulator,
        operator: OperatorInterface,
        payload: PayloadInterface,
        landing_markers: LandingMarkerInterface,
        test_targets: TestTargetInterface,
        target_markers: TargetMarkerPublisher,
    ):
        # Flight control is supplied by the entry point rather than created
        # internally. This keeps mission decisions separate from ROS/SPAR
        # communication and allows the same controller to use a simulated
        # flight interface during testing.
        self._flight = flight

        # Breadcrumb supplies obstacle-aware routes while the mission
        # controller remains responsible for choosing destinations.
        self._breadcrumb = breadcrumb

        # VehicleState supplies validated pose and battery facts. Mission
        # policy remains within this controller.
        self._vehicle = vehicle

        # Raw person and backpack observations arrive through a dedicated
        # thread-safe boundary. Evidence and mission policy remain separate.
        self._target_observations = target_observations

        # The accumulator converts raw observations into decaying,
        # class-specific spatial evidence. The controller will separately
        # decide when that evidence is sufficient to change mission state.
        self._target_evidence = target_evidence
        self._target_evidence_started_at = None

        # OperatorInterface owns Ground Control communication only. Whether to
        # offer a veto, continue autonomously or reject deployment remains
        # mission policy in this controller.
        self._operator = operator

        # PayloadInterface owns actuator topics and acknowledgements. The
        # controller owns target-to-payload mission policy and flight motion.
        self._payload = payload

        # LandingMarkerInterface retains the persistent ArUco world model and
        # owns terminal input. The controller remains responsible for hover,
        # approach routing and the final landing decision.
        self._landing_markers = landing_markers

        # TestTargetInterface is deliberately separate from detector evidence.
        # An explicitly enabled service can exercise route diversion and the
        # normal deployment sequence without pretending that YOLO saw an
        # object or changing the production confirmation thresholds.
        self._test_targets = test_targets
        self._injected_target_location = None

        # TargetMarkerPublisher displays the evidence already owned by the
        # accumulator. Visualisation remains downstream of mission evidence and
        # cannot create, confirm or modify target candidates.
        self._target_markers = target_markers

        # Initial confirmation allows investigation through DIVERT; it does
        # not authorise payload deployment. Person and backpack thresholds
        # remain independently configurable because their measured detector
        # reliability may differ.
        self._initial_evidence_thresholds = {
            TargetObservationBuffer.PERSON_CLASS_ID: rospy.get_param(
                "~person_initial_evidence_threshold", 2.5
            ),
            TargetObservationBuffer.BACKPACK_CLASS_ID: rospy.get_param(
                "~backpack_initial_evidence_threshold", 2.5
            ),
        }
        self._candidate_freshness = rospy.get_param(
            "~target_candidate_freshness", 1.0
        )
        self._minimum_candidate_observations = int(
            rospy.get_param("~target_minimum_observations", 3)
        )

        for class_id, threshold in self._initial_evidence_thresholds.items():
            if not math.isfinite(threshold) or threshold <= 0:
                raise ValueError(
                    "Initial evidence threshold for class {} must be finite and greater than zero".format(
                        class_id
                    )
                )

        if not math.isfinite(self._candidate_freshness) or self._candidate_freshness <= 0:
            raise ValueError(
                "~target_candidate_freshness must be finite and greater than zero"
            )

        if self._minimum_candidate_observations <= 0:
            raise ValueError(
                "~target_minimum_observations must be greater than zero"
            )

        # A confirmed candidate is retained by identity while the aircraft
        # diverts, centres and reassesses it. The current evidence snapshot is
        # retrieved by ID so its score and estimated position may keep changing.
        self._active_candidate_id = None
        self._active_candidate_class_id = None

        # Centring uses normalised image coordinates, where (0.5, 0.5) is the
        # image centre. The target must remain inside the tolerance region
        # using fresh observations for the complete hold duration.
        self._centre_tolerance_x = rospy.get_param(
            "~target_centre_tolerance_x", 0.10
        )
        self._centre_tolerance_y = rospy.get_param(
            "~target_centre_tolerance_y", 0.10
        )
        self._centre_hold_time = rospy.get_param(
            "~target_centre_hold_time", 1.0
        )
        self._centre_observation_freshness = rospy.get_param(
            "~target_centre_observation_freshness", 0.5
        )
        self._centre_timeout = rospy.get_param(
            "~target_centre_timeout", 10.0
        )
        self._diversion_timeout = rospy.get_param(
            "~target_diversion_timeout", 20.0
        )
        self._candidate_centred_since = None

        for name, tolerance in (
            ("~target_centre_tolerance_x", self._centre_tolerance_x),
            ("~target_centre_tolerance_y", self._centre_tolerance_y),
        ):
            if not math.isfinite(tolerance) or not 0 < tolerance < 0.5:
                raise ValueError(
                    "{} must be finite and between zero and 0.5".format(name)
                )

        for name, duration in (
            ("~target_centre_hold_time", self._centre_hold_time),
            (
                "~target_centre_observation_freshness",
                self._centre_observation_freshness,
            ),
            ("~target_centre_timeout", self._centre_timeout),
            ("~target_diversion_timeout", self._diversion_timeout),
        ):
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        if self._centre_observation_freshness >= self._centre_hold_time:
            raise ValueError(
                "~target_centre_observation_freshness must be shorter than ~target_centre_hold_time"
            )

        if self._centre_timeout <= self._centre_hold_time:
            raise ValueError(
                "~target_centre_timeout must exceed ~target_centre_hold_time"
            )

        # ROS callbacks and the mission timer may run concurrently, so state
        # reads and transitions share one re-entrant lock.
        self._lock = threading.RLock()

        # A finite connection timeout prevents mission startup from hanging
        # indefinitely when SPAR is unavailable.
        self._dependency_timeout = rospy.get_param(
            "~dependency_timeout",
            30.0,
        )

        # Brief OptiTrack interruptions pause position-dependent progression.
        # A continuous ten-second outage triggers the abort-landing path.
        self._optitrack_loss_timeout = rospy.get_param(
            "~optitrack_loss_timeout",
            10.0,
        )

        if (
            not math.isfinite(self._optitrack_loss_timeout)
            or self._optitrack_loss_timeout <= 0
        ):
            raise ValueError(
                "~optitrack_loss_timeout must be finite and greater than zero"
            )

        # Preserve the established survey-altitude parameter so existing
        # launch configurations remain compatible.
        self._survey_altitude = rospy.get_param(
            "~surveyAlt",
            2.0,
        )

        if (
            not math.isfinite(self._survey_altitude)
            or self._survey_altitude <= 0
        ):
            raise ValueError(
                "~surveyAlt must be finite and greater than zero"
            )

        # Use the established SPAR position tolerance when deciding whether a
        # high-level search destination has already been reached.
        self._search_destination_tolerance = rospy.get_param(
            "~acc_pos",
            0.3,
        )

        if (
            not math.isfinite(self._search_destination_tolerance)
            or self._search_destination_tolerance <= 0
        ):
            raise ValueError(
                "~acc_pos must be finite and greater than zero"
            )

        # High-level destinations describe the established lawnmower search.
        # Breadcrumb will calculate each obstacle-aware route during flight.
        self._search_waypoints = build_default_search_plan(
            self._survey_altitude
        )

        # Preserve the established RViz/GCS path topics used by the legacy
        # Breadcrumb mission. These publishers report the high-level search
        # plan and the currently selected obstacle-avoiding route; they do not
        # influence flight control.
        self._full_path_publisher = rospy.Publisher(
            "/guidance/fullPath", Path, queue_size=10, latch=True
        )
        self._breadcrumb_path_publisher = rospy.Publisher(
            "/guidance/pathBreadcrumb", Path, queue_size=10, latch=True
        )
        self._publish_visualisation_path(
            self._search_waypoints, self._full_path_publisher
        )

        self._search_waypoint_index = 0

        # Preserve the established map-frame mission boundary. It guards every
        # commanded navigation waypoint when explicitly enabled.
        self._boundary_checks_enabled = rospy.get_param(
            "~boundary_checks_enabled",
            True,
        )

        self._x_min = rospy.get_param("~x_min", -4.0)
        self._x_max = rospy.get_param("~x_max", 4.0)
        self._y_min = rospy.get_param("~y_min", -3.2)
        self._y_max = rospy.get_param("~y_max", 2.0)
        self._z_min = rospy.get_param("~z_min", 0.5)
        self._z_max = rospy.get_param("~z_max", 4.0)

        for lower, upper, axis_name in (
            (self._x_min, self._x_max, "x"),
            (self._y_min, self._y_max, "y"),
            (self._z_min, self._z_max, "z"),
        ):
            if (
                not math.isfinite(lower)
                or not math.isfinite(upper)
                or lower >= upper
            ):
                raise ValueError(
                    "Invalid {}-axis mission boundary".format(
                        axis_name
                    )
                )

        if not self._boundary_checks_enabled:
            rospy.logwarn(
                "Mission waypoint boundary checks are disabled"
            )
        else:
            rospy.loginfo(
                "Mission boundary checks enabled: x [%.1f, %.1f], y [%.1f, %.1f], z [%.1f, %.1f]",
                self._x_min,
                self._x_max,
                self._y_min,
                self._y_max,
                self._z_min,
                self._z_max,
            )

        # During take-off the aircraft must be allowed to climb upward through
        # z_min. After reaching z_min once, descending below it outside LAND or
        # ABORT is treated as a boundary violation.
        self._minimum_altitude_reached = False

        # Breadcrumb expands one high-level search destination into a list of
        # obstacle-aware intermediate waypoints. These fields record the
        # active leg without changing the underlying lawnmower plan.
        self._active_route = []
        self._route_waypoint_index = 0

        # Diversion uses a separate Breadcrumb route so interrupting a search
        # leg cannot make its remaining waypoints appear to be diversion
        # waypoints. Returning to search will replan the interrupted search leg.
        self._diversion_route = []
        self._diversion_waypoint_index = 0
        self._diversion_waypoint_active = False
        self._next_diversion_route_request_at = 0.0

        # Centring corrections also use Breadcrumb so image-guided adjustments
        # do not bypass the established obstacle-avoidance system.
        self._centring_route = []
        self._centring_waypoint_index = 0
        self._centring_waypoint_active = False
        self._next_centring_route_request_at = 0.0

        # Normal mission completion retains the legacy operator-selected
        # ArUco landing workflow. The aircraft first travels to the selected
        # marker at survey height, then descends above it before LAND.
        self._landing_approach_altitude = rospy.get_param("~landing_approach_altitude", 0.8)
        if not math.isfinite(self._landing_approach_altitude) or self._landing_approach_altitude <= 0:
            raise ValueError("~landing_approach_altitude must be finite and greater than zero")
        if self._landing_approach_altitude >= self._survey_altitude:
            raise ValueError("~landing_approach_altitude must be below ~surveyAlt")

        self._selected_landing_marker: Optional[LandingMarker] = None
        self._landing_route = []
        self._landing_route_index = 0
        self._landing_waypoint_active = False
        self._landing_approach_phase = LandingApproachPhase.PLANNING
        self._landing_descent_command_sent = False
        self._next_landing_route_request_at = 0.0

        # Final confirmation is intentionally stricter than the initial
        # diversion decision. It requires additional observations collected
        # after centring, fresh image alignment and a class-specific score.
        self._final_evidence_thresholds = {
            TargetObservationBuffer.PERSON_CLASS_ID: rospy.get_param(
                "~person_final_evidence_threshold", 3.5
            ),
            TargetObservationBuffer.BACKPACK_CLASS_ID: rospy.get_param(
                "~backpack_final_evidence_threshold", 3.5
            ),
        }
        self._reassessment_minimum_observations = int(
            rospy.get_param("~target_reassessment_minimum_observations", 3)
        )
        self._reassessment_hold_time = rospy.get_param(
            "~target_reassessment_hold_time", 1.0
        )
        self._reassessment_timeout = rospy.get_param(
            "~target_reassessment_timeout", 5.0
        )
        self._reassessment_baseline_observations = None
        self._reassessment_confirmed_since = None

        for class_id, threshold in self._final_evidence_thresholds.items():
            if not math.isfinite(threshold) or threshold <= 0:
                raise ValueError(
                    "Final evidence threshold for class {} must be finite and greater than zero".format(
                        class_id
                    )
                )

            if threshold < self._initial_evidence_thresholds[class_id]:
                raise ValueError(
                    "Final evidence threshold for class {} cannot be lower than its initial threshold".format(
                        class_id
                    )
                )

        if self._reassessment_minimum_observations <= 0:
            raise ValueError(
                "~target_reassessment_minimum_observations must be greater than zero"
            )

        for name, duration in (
            ("~target_reassessment_hold_time", self._reassessment_hold_time),
            ("~target_reassessment_timeout", self._reassessment_timeout),
        ):
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        if self._reassessment_timeout <= self._reassessment_hold_time:
            raise ValueError(
                "~target_reassessment_timeout must exceed ~target_reassessment_hold_time"
            )

        # Ground Control is considered useful for a veto only while it is
        # acknowledging newly received images. Loss of this optional path must
        # not prevent autonomous deployment after final confirmation.
        self._gcs_image_freshness = rospy.get_param(
            "~gcs_image_freshness", 1.0
        )
        self._operator_veto_timeout = rospy.get_param(
            "~operator_veto_timeout", 5.0
        )
        self._veto_request_started_at = None
        self._veto_request_sent = False

        for name, duration in (
            ("~gcs_image_freshness", self._gcs_image_freshness),
            ("~operator_veto_timeout", self._operator_veto_timeout),
        ):
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        # A veto permanently excludes that candidate area for the remainder
        # of this mission. Other unsuccessful investigations receive a short
        # retry delay so persistent detections cannot immediately form a loop.
        self._target_blacklist_radius = rospy.get_param(
            "~target_blacklist_radius", 1.0
        )
        self._target_retry_delay = rospy.get_param(
            "~target_retry_delay", 5.0
        )

        for name, value in (
            ("~target_blacklist_radius", self._target_blacklist_radius),
            ("~target_retry_delay", self._target_retry_delay),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        self._blacklisted_target_locations = {
            TargetObservationBuffer.PERSON_CLASS_ID: [],
            TargetObservationBuffer.BACKPACK_CLASS_ID: [],
        }
        self._candidate_retry_after = {}

        # Preserve the legacy 0.8 m release height as an explicit parameter.
        # DEPLOY performs descent, acknowledged actuation and return to survey
        # altitude without blocking the 20 Hz mission dispatcher.
        self._deployment_altitude = rospy.get_param(
            "~deployment_altitude", 0.8
        )
        self._payload_ack_timeout = rospy.get_param(
            "~payload_ack_timeout", 3.0
        )

        for name, value in (
            ("~deployment_altitude", self._deployment_altitude),
            ("~payload_ack_timeout", self._payload_ack_timeout),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        if self._deployment_altitude >= self._survey_altitude:
            raise ValueError(
                "~deployment_altitude must be below ~surveyAlt"
            )

        self._deployment_phase = DeploymentPhase.IDLE
        self._deployment_commanded_at = None
        self._deployment_position = None

        # Failed or temporarily unavailable route requests are retried without
        # calling the Breadcrumb service on every 20 Hz controller update.
        self._route_retry_delay = rospy.get_param(
            "~breadcrumb_retry_delay",
            1.0,
        )

        if (
            not math.isfinite(self._route_retry_delay)
            or self._route_retry_delay <= 0
        ):
            raise ValueError(
                "~breadcrumb_retry_delay must be finite and greater than zero"
            )

        self._next_route_request_at = 0.0

        # The 20 Hz state handler may run many times while take-off is active.
        # This flag prevents it from submitting the same command repeatedly.
        self._takeoff_command_sent = False

        # Normal landing and emergency abort landing are supervised
        # independently so a failed normal landing can still trigger one
        # explicit abort-landing attempt.
        self._landing_command_sent = False
        self._abort_landing_attempted = False
        self._abort_landing_submitted = False

        # COMPLETE requests ROS shutdown once. The flag prevents repeated
        # shutdown requests while the 20 Hz handler is still being retired.
        self._completion_handled = False

        if self._dependency_timeout <= 0:
            raise ValueError(
                "~dependency_timeout must be greater than zero"
            )

        # The controller always begins in a state that cannot command motion.
        # TAKEOFF will only become legal after explicit preflight success.
        self._state = MissionState.PREFLIGHT

        self._state_entered_at = time.monotonic()

        self._update_rate = rospy.get_param(
            "~mission_update_rate",
            20.0,
        )

        if self._update_rate <= 0:
            raise ValueError(
                "~mission_update_rate must be greater than zero"
            )

        self._timer = None

        # Each state has one authoritative handler. Sensor callbacks update
        # observations; these handlers decide whether mission state changes.
        self._state_handlers = {
            MissionState.PREFLIGHT: self._handle_preflight,
            MissionState.TAKEOFF: self._handle_takeoff,
            MissionState.SEARCH: self._handle_search,
            MissionState.DIVERT: self._handle_divert,
            MissionState.CENTRE: self._handle_centre,
            MissionState.REASSESS: self._handle_reassess,
            MissionState.WAIT_FOR_VETO: self._handle_wait_for_veto,
            MissionState.DEPLOY: self._handle_deploy,
            MissionState.RETURN_TO_SEARCH: self._handle_return_to_search,
            MissionState.WAIT_FOR_LANDING_SELECTION: self._handle_wait_for_landing_selection,
            MissionState.APPROACH_LANDING: self._handle_approach_landing,
            MissionState.LAND: self._handle_land,
            MissionState.COMPLETE: self._handle_complete,
            MissionState.ABORT: self._handle_abort,
        }

        self._test_targets.set_request_handler(self._inject_test_target)

        rospy.loginfo(
            "Autonomous mission created in state: %s",
            self._state.name,
        )

    @property
    def state(self) -> MissionState:
        """Return the currently active mission state."""

        with self._lock:
            return self._state

    def _inject_test_target(self, class_id: int, class_name: str):
        """Accept a gated test target and begin an immediate diversion."""
        with self._lock:
            if self._state != MissionState.SEARCH:
                return False, "test targets are accepted only while the mission is searching", None

            if not self._payload.class_is_available(class_id):
                return False, "the {} payload is no longer available".format(class_name), None

            current_position = self._vehicle.current_position()
            if current_position is None:
                return False, "vehicle pose is not fresh", None

            if not self._vehicle.battery_is_ready():
                return False, "battery telemetry is unavailable or unsafe", None

            search_x = [waypoint.x for waypoint in self._search_waypoints]
            search_y = [waypoint.y for waypoint in self._search_waypoints]
            location = self._test_targets.random_free_location(min(search_x), max(search_x), min(search_y), max(search_y), current_position.x, current_position.y)

            if location is None:
                return False, "no obstacle-free location is available within 2 m and inside the search footprint", None

            self._active_candidate_id = None
            self._active_candidate_class_id = class_id
            self._injected_target_location = location
            self._candidate_centred_since = None

            if self._flight.goal_state() in (FlightGoalState.PENDING, FlightGoalState.ACTIVE):
                self._flight.cancel_current_goal()

            self._active_route = []
            self._route_waypoint_index = 0
            self._next_route_request_at = 0.0
            self._diversion_route = []
            self._diversion_waypoint_index = 0
            self._diversion_waypoint_active = False
            self._next_diversion_route_request_at = 0.0

            if not self.transition_to(MissionState.DIVERT, "explicit {} test target injected at [{:.2f}, {:.2f}]".format(class_name, location[0], location[1])):
                self._injected_target_location = None
                self._active_candidate_class_id = None
                return False, "mission rejected the test-target diversion", None

        message = "accepted {} test target at [{:.2f}, {:.2f}]".format(class_name, location[0], location[1])
        rospy.logwarn("%s; visual confirmation and operator veto are bypassed for this explicit test", message)
        return True, message, location

    def perform_preflight(self) -> bool:
        """
        Check whether the dependencies required for safe mission startup exist.

        PREFLIGHT remains active until every prerequisite passes. An
        unavailable required dependency places the mission in ABORT without
        sending an aircraft command.

        Returns:
            True when all implemented preflight checks pass.
            False when a required dependency is unavailable.
        """

        rospy.loginfo("Starting autonomous mission preflight checks")

        if not self._flight.wait_until_ready(
            self._dependency_timeout
        ):
            self.transition_to(
                MissionState.ABORT,
                "SPAR flight-action server is unavailable",
            )
            return False

        if not self._breadcrumb.wait_until_ready(
            self._dependency_timeout
        ):
            self.transition_to(
                MissionState.ABORT,
                "Breadcrumb path-planning service is unavailable",
            )
            return False

        if not self._vehicle.wait_until_ready(
            self._dependency_timeout
        ):
            self.transition_to(
                MissionState.ABORT,
                "Vehicle pose or battery telemetry is not ready",
            )
            return False

        if not self._payload.wait_until_ready(self._dependency_timeout):
            self.transition_to(
                MissionState.ABORT,
                "payload controller is unavailable",
            )
            return False

        rospy.loginfo(
            "Preflight passed: required mission services available"
        )
        return True

    def transition_to(
        self,
        next_state: MissionState,
        reason: str,
    ) -> bool:
        """
        Attempt a guarded mission-state transition.

        Every transition passes through this method so state changes are
        validated, timestamped and recorded consistently.

        Returns:
            True when the state changed.
            False when the request was redundant or prohibited.
        """

        if not isinstance(next_state, MissionState):
            raise TypeError("next_state must be a MissionState")

        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("A non-empty transition reason is required")

        with self._lock:
            if next_state == self._state:
                rospy.logdebug(
                    "Mission remains in state %s: %s",
                    self._state.name,
                    reason,
                )
                return False

            permitted_states = self.LEGAL_TRANSITIONS[self._state]

            if next_state not in permitted_states:
                rospy.logerr(
                    "Illegal mission transition %s -> %s: %s",
                    self._state.name,
                    next_state.name,
                    reason,
                )
                return False

            previous_state = self._state
            self._state = next_state
            self._state_entered_at = time.monotonic()

        rospy.loginfo(
            "Mission transition %s -> %s: %s",
            previous_state.name,
            next_state.name,
            reason,
        )
        return True

    def _live_boundary_allows_update(
        self,
        current_state: MissionState,
    ) -> bool:
        """
        Check the measured aircraft position against the mission boundary.

        Initial take-off may pass upward through z_min. LAND and ABORT are
        excluded because both must be permitted to descend below z_min.
        """

        if not self._boundary_checks_enabled:
            return True

        monitored_states = {
            MissionState.TAKEOFF,
            MissionState.SEARCH,
            MissionState.DIVERT,
            MissionState.CENTRE,
            MissionState.REASSESS,
            MissionState.WAIT_FOR_VETO,
            MissionState.DEPLOY,
            MissionState.RETURN_TO_SEARCH,
            MissionState.WAIT_FOR_LANDING_SELECTION,
            MissionState.APPROACH_LANDING,
        }

        if current_state not in monitored_states:
            return True

        current_position = self._vehicle.current_position()

        # Pose-loss policy is handled by the separate OptiTrack watchdog.
        if current_position is None:
            return True

        if current_position.z >= self._z_min:
            self._minimum_altitude_reached = True

        outside_boundary = (
            current_position.x < self._x_min
            or current_position.x > self._x_max
            or current_position.y < self._y_min
            or current_position.y > self._y_max
            or current_position.z > self._z_max
            or (
                self._minimum_altitude_reached
                and current_position.z < self._z_min
            )
        )

        if not outside_boundary:
            return True

        self.transition_to(
            MissionState.ABORT,
            "aircraft position crossed the configured mission boundary",
        )
        return False

    def _runtime_safety_allows_update(
        self,
        current_state: MissionState,
    ) -> bool:
        """
        Enforce the continuous external-position safety requirement.

        Position-dependent operations may pause earlier when an individual
        pose becomes stale. Mission abort occurs only after the direct
        external-position source has been absent for ten continuous seconds.
        """

        monitored_states = {
            MissionState.TAKEOFF,
            MissionState.SEARCH,
            MissionState.DIVERT,
            MissionState.CENTRE,
            MissionState.REASSESS,
            MissionState.WAIT_FOR_VETO,
            MissionState.DEPLOY,
            MissionState.RETURN_TO_SEARCH,
            MissionState.WAIT_FOR_LANDING_SELECTION,
            MissionState.APPROACH_LANDING,
        }

        if current_state not in monitored_states:
            return True

        external_position_age = (
            self._vehicle.external_position_age()
        )

        if external_position_age is None:
            # Preflight normally prevents this case. Measuring from state entry
            # still provides a complete grace period if no observation exists.
            with self._lock:
                missing_duration = (
                    time.monotonic() - self._state_entered_at
                )
        else:
            missing_duration = external_position_age

        if missing_duration < self._optitrack_loss_timeout:
            if missing_duration > 1.0:
                rospy.logwarn_throttle(
                    2.0,
                    "External positioning has been unavailable for %.1f s",
                    missing_duration,
                )

            return True

        self.transition_to(
            MissionState.ABORT,
            "OptiTrack unavailable continuously for {:.1f} seconds".format(
                missing_duration
            ),
        )
        return False

    def _update_target_evidence(self, current_state: MissionState):
        """
        Process target evidence only during target-relevant mission phases.

        Detections seen during PREFLIGHT or TAKEOFF cannot later trigger a
        diversion. Once SEARCH begins, evidence continues through diversion,
        centring, reassessment and return-to-search.
        """

        evidence_states = {
            MissionState.SEARCH,
            MissionState.DIVERT,
            MissionState.CENTRE,
            MissionState.REASSESS,
            MissionState.WAIT_FOR_VETO,
            MissionState.DEPLOY,
            MissionState.RETURN_TO_SEARCH,
        }

        if current_state not in evidence_states:
            return

        now = time.monotonic()

        if self._target_evidence_started_at is None:
            self._target_evidence_started_at = now

        frame = self._target_observations.latest_frame()

        if (
            frame is not None
            and frame.received_at >= self._target_evidence_started_at
        ):
            self._target_evidence.process_frame(frame)

        self._target_evidence.advance_time(now)

    def _strongest_confirmed_candidate(self) -> Optional[EvidenceCandidate]:
        """
        Return the strongest fresh candidate satisfying initial confirmation.

        Candidates are compared by score relative to their class-specific
        threshold so person and backpack thresholds may later differ fairly.
        Initial confirmation permits investigation, not deployment.
        """

        now = time.monotonic()
        confirmed_candidates = []

        for candidate in self._target_evidence.candidates():
            threshold = self._initial_evidence_thresholds.get(
                candidate.class_id
            )

            if threshold is None:
                continue

            if not self._payload.class_is_available(candidate.class_id):
                continue

            if self._candidate_is_blacklisted(candidate):
                continue

            if now < self._candidate_retry_after.get(candidate.candidate_id, 0.0):
                continue

            if candidate.observation_count < self._minimum_candidate_observations:
                continue

            if now - candidate.last_seen_seconds > self._candidate_freshness:
                continue

            if candidate.score < threshold:
                continue

            confirmed_candidates.append(candidate)

        if not confirmed_candidates:
            return None

        return max(
            confirmed_candidates,
            key=lambda candidate: (
                candidate.score
                / self._initial_evidence_thresholds[candidate.class_id],
                candidate.score,
                candidate.observation_count,
            ),
        )

    def _active_candidate(self) -> Optional[EvidenceCandidate]:
        """Return the latest snapshot of the active candidate, if available."""

        if self._active_candidate_id is None:
            return None

        for candidate in self._target_evidence.candidates():
            if candidate.candidate_id == self._active_candidate_id:
                return candidate

        return None

    def _select_active_candidate(self, candidate: EvidenceCandidate):
        """
        Retain the confirmed candidate selected for investigation.

        Selection alone does not command the aircraft or change mission state.
        """

        if not isinstance(candidate, EvidenceCandidate):
            raise TypeError("candidate must be an EvidenceCandidate")

        self._active_candidate_id = candidate.candidate_id
        self._active_candidate_class_id = candidate.class_id
        self._candidate_centred_since = None

        rospy.loginfo(
            "Selected target candidate %i, class %i, score %.2f at [%.2f, %.2f, %.2f]",
            candidate.candidate_id,
            candidate.class_id,
            candidate.score,
            candidate.map_x,
            candidate.map_y,
            candidate.map_z,
        )

    def _active_candidate_is_stably_centred(
        self,
        candidate: EvidenceCandidate,
    ) -> bool:
        """
        Return whether fresh observations remain centred for the hold time.

        A stale observation cannot complete the hold period. This prevents one
        well-positioned frame from being reused repeatedly by the 20 Hz timer.
        """

        now = time.monotonic()
        observation_age = now - candidate.last_seen_seconds

        if observation_age > self._centre_observation_freshness:
            self._candidate_centred_since = None
            return False

        offset_x = candidate.bbox_center_x_normalized - 0.5
        offset_y = candidate.bbox_center_y_normalized - 0.5

        within_tolerance = (
            abs(offset_x) <= self._centre_tolerance_x
            and abs(offset_y) <= self._centre_tolerance_y
        )

        if not within_tolerance:
            self._candidate_centred_since = None
            return False

        if self._candidate_centred_since is None:
            self._candidate_centred_since = now
            rospy.loginfo(
                "Candidate %i entered the image-centre tolerance",
                candidate.candidate_id,
            )
            return False

        return (
            now - self._candidate_centred_since
            >= self._centre_hold_time
        )

    def _active_candidate_passes_final_confirmation(
        self,
        candidate: EvidenceCandidate,
    ) -> bool:
        """
        Return whether the centred candidate passes final autonomous review.

        The required observations must be collected after entering REASSESS.
        This prevents the evidence used to justify diversion from being reused
        as the complete final deployment decision.
        """

        if self._reassessment_baseline_observations is None:
            return False

        now = time.monotonic()
        threshold = self._final_evidence_thresholds.get(candidate.class_id)

        if threshold is None:
            return False

        new_observations = (
            candidate.observation_count
            - self._reassessment_baseline_observations
        )
        observation_is_fresh = (
            now - candidate.last_seen_seconds
            <= self._centre_observation_freshness
        )
        remains_centred = (
            abs(candidate.bbox_center_x_normalized - 0.5)
            <= self._centre_tolerance_x
            and abs(candidate.bbox_center_y_normalized - 0.5)
            <= self._centre_tolerance_y
        )

        confirmation_conditions_pass = (
            new_observations >= self._reassessment_minimum_observations
            and observation_is_fresh
            and remains_centred
            and candidate.score >= threshold
        )

        if not confirmation_conditions_pass:
            self._reassessment_confirmed_since = None
            return False

        if self._reassessment_confirmed_since is None:
            self._reassessment_confirmed_since = now
            rospy.loginfo(
                "Candidate %i entered final confirmation",
                candidate.candidate_id,
            )
            return False

        return (
            now - self._reassessment_confirmed_since
            >= self._reassessment_hold_time
        )

    def _candidate_is_blacklisted(self, candidate: EvidenceCandidate) -> bool:
        """Return whether a mission-long veto exclusion covers the candidate."""
        locations = self._blacklisted_target_locations.get(candidate.class_id, ())

        return any(
            math.hypot(candidate.map_x - x, candidate.map_y - y)
            <= self._target_blacklist_radius
            for x, y in locations
        )

    def _blacklist_active_candidate(self):
        """Exclude the active candidate location for the rest of the mission."""
        candidate = self._active_candidate()

        if candidate is None:
            return

        locations = self._blacklisted_target_locations[candidate.class_id]
        locations.append((candidate.map_x, candidate.map_y))
        rospy.logwarn(
            "Blacklisted candidate %i location [%.2f, %.2f] for class %i",
            candidate.candidate_id,
            candidate.map_x,
            candidate.map_y,
            candidate.class_id,
        )

    def _suppress_active_candidate_temporarily(self):
        """Prevent an unsuccessful candidate from immediately looping."""
        if self._active_candidate_id is None:
            return

        self._candidate_retry_after[self._active_candidate_id] = (
            time.monotonic() + self._target_retry_delay
        )

    def _candidate_remains_deployable(
        self,
        candidate: Optional[EvidenceCandidate],
    ) -> bool:
        """Return whether final visual, payload and battery gates still pass."""
        if candidate is None or candidate.class_id != self._active_candidate_class_id:
            return False

        threshold = self._final_evidence_thresholds.get(candidate.class_id)

        if threshold is None or candidate.score < threshold:
            return False

        if time.monotonic() - candidate.last_seen_seconds > self._centre_observation_freshness:
            return False

        if (
            abs(candidate.bbox_center_x_normalized - 0.5) > self._centre_tolerance_x
            or abs(candidate.bbox_center_y_normalized - 0.5) > self._centre_tolerance_y
        ):
            return False

        return self._payload.class_is_available(candidate.class_id) and self._vehicle.battery_is_ready()

    def _clear_target_handling(self):
        """Reset transient investigation state while preserving search progress."""
        self._operator.clear_request()
        self._active_candidate_id = None
        self._active_candidate_class_id = None
        self._injected_target_location = None
        self._candidate_centred_since = None
        self._reassessment_baseline_observations = None
        self._reassessment_confirmed_since = None
        self._veto_request_started_at = None
        self._veto_request_sent = False
        self._diversion_route = []
        self._diversion_waypoint_index = 0
        self._diversion_waypoint_active = False
        self._centring_route = []
        self._centring_waypoint_index = 0
        self._centring_waypoint_active = False
        self._deployment_phase = DeploymentPhase.IDLE
        self._deployment_commanded_at = None
        self._deployment_position = None

    def _update(self, timer_event):
        """
        Dispatch one periodic update to the active mission-state handler.

        The legacy mission used one large timer callback,
        check_waypoint_status(), for route following, waypoint progression,
        payload decisions and other mission behaviour. This controller keeps
        the same periodic-update concept but separates each state's behaviour
        into a dedicated handler.

        Keeping dispatch separate from behaviour makes the active state
        explicit and prevents unrelated mission decisions from accumulating
        in one callback.
        """

        del timer_event

        if rospy.is_shutdown():
            return

        with self._lock:
            current_state = self._state
            handler = self._state_handlers[current_state]

        self._update_target_evidence(current_state)

        # Publish the complete remembered target set after processing the
        # latest detector frame. An injected target is included separately
        # because test injection deliberately bypasses detector evidence.
        self._target_markers.publish(
            candidates=self._target_evidence.candidates(),
            injected_class_id=self._active_candidate_class_id,
            injected_location=self._injected_target_location,
        )

        if not self._runtime_safety_allows_update(
            current_state
        ):
            return

        if not self._live_boundary_allows_update(
            current_state
        ):
            return

        # Release the lock before invoking the handler because the handler may
        # call transition_to(), which also controls access to mission state.
        handler()

    def _handle_preflight(self):
        """Hold mission authority until every startup prerequisite passes."""
        pass

    def _handle_takeoff(self):
        """
        Submit one take-off command and supervise it until completion.

        The handler runs repeatedly at the mission update rate. It sends the
        command once, then uses SPAR's reported goal state to decide whether
        the mission may enter SEARCH or must enter ABORT.
        """

        if not self._takeoff_command_sent:
            # Recheck the immediately relevant safety information because
            # telemetry may have changed since PREFLIGHT completed.
            if not self._vehicle.pose_is_ready():
                self.transition_to(
                    MissionState.ABORT,
                    "vehicle pose became unavailable before take-off",
                )
                return

            if not self._vehicle.battery_is_ready():
                self.transition_to(
                    MissionState.ABORT,
                    "battery became unavailable or unsafe before take-off",
                )
                return

            if not self._flight.send_takeoff(
                self._survey_altitude
            ):
                self.transition_to(
                    MissionState.ABORT,
                    "SPAR rejected the take-off command",
                )
                return

            self._takeoff_command_sent = True
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            self.transition_to(
                MissionState.SEARCH,
                "take-off completed at the configured search altitude",
            )
            return

        if goal_state == FlightGoalState.CANCELLED:
            self.transition_to(
                MissionState.ABORT,
                "take-off command was cancelled",
            )
            return

        self.transition_to(
            MissionState.ABORT,
            "take-off command failed or returned an invalid state",
        )

    def _waypoint_is_within_boundary(
        self,
        waypoint: Waypoint,
    ) -> bool:
        """
        Return whether a navigation waypoint satisfies the mission boundary.

        Waypoint already guarantees finite coordinates. When boundary checking
        is disabled, the waypoint is accepted to preserve the current deployed
        configuration.
        """

        if not isinstance(waypoint, Waypoint):
            raise TypeError("waypoint must be a Waypoint")

        if not self._boundary_checks_enabled:
            return True

        return (
            self._x_min <= waypoint.x <= self._x_max
            and self._y_min <= waypoint.y <= self._y_max
            and self._z_min <= waypoint.z <= self._z_max
        )

    @staticmethod
    def _publish_visualisation_path(waypoints, publisher) -> None:
        """Publish a waypoint sequence using the established GUI path format."""

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "/map"

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = waypoint.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        publisher.publish(path)

    def _request_current_search_route(self) -> bool:
        """
        Request an obstacle-aware route to the current search destination.

        The high-level destination comes from the established lawnmower plan.
        The route begins at the latest fresh vehicle position rather than an
        assumed previous waypoint.

        Returns:
            True when a usable route was stored.
            False when the request cannot currently be completed.
        """

        now = time.monotonic()

        if now < self._next_route_request_at:
            return False

        self._next_route_request_at = (
            now + self._route_retry_delay
        )

        if self._search_waypoint_index >= len(
            self._search_waypoints
        ):
            return False

        current_position = self._vehicle.current_position()

        if current_position is None:
            rospy.logwarn_throttle(
                2.0,
                "Cannot request search route without a fresh vehicle pose",
            )
            return False

        destination = self._search_waypoints[
            self._search_waypoint_index
        ]

        destination_point = Point(
            x=destination.x,
            y=destination.y,
            z=destination.z,
        )

        rospy.loginfo(
            "Planning search leg %i of %i",
            self._search_waypoint_index + 1,
            len(self._search_waypoints),
        )

        route = self._breadcrumb.request_path(
            current_position,
            destination_point,
        )

        if not route:
            rospy.logerr_throttle(
                2.0,
                "No usable Breadcrumb route to search destination %i",
                self._search_waypoint_index + 1,
            )
            return False

        self._active_route = route
        self._route_waypoint_index = 0
        self._next_route_request_at = 0.0
        self._publish_visualisation_path(
            self._active_route, self._breadcrumb_path_publisher
        )

        rospy.loginfo(
            "Stored Breadcrumb search route containing %i waypoints",
            len(self._active_route),
        )
        return True

    def _send_next_route_waypoint(self) -> bool:
        """
        Submit the next waypoint in the active Breadcrumb route.

        The route index advances only after SPAR accepts the command. A stale
        pose pauses route progression without immediately aborting the mission;
        the separate telemetry-loss policy determines whether a prolonged
        OptiTrack outage requires an abort.

        Returns:
            True when a waypoint was submitted.
            False when no waypoint was submitted.
        """

        if self._route_waypoint_index >= len(
            self._active_route
        ):
            return False

        if self._vehicle.current_position() is None:
            rospy.logwarn_throttle(
                2.0,
                "Holding search route because vehicle pose is not fresh",
            )
            return False

        if not self._vehicle.battery_is_ready():
            self.transition_to(
                MissionState.ABORT,
                "battery telemetry is unavailable or unsafe",
            )
            return False

        waypoint = self._active_route[
            self._route_waypoint_index
        ]

        if not self._waypoint_is_within_boundary(
            waypoint
        ):
            self.transition_to(
                MissionState.ABORT,
                "Breadcrumb produced a waypoint outside the mission boundary",
            )
            return False

        if not self._flight.send_waypoint(
            waypoint
        ):
            self.transition_to(
                MissionState.ABORT,
                "SPAR rejected a Breadcrumb waypoint",
            )
            return False

        self._route_waypoint_index += 1
        return True

    def _request_diversion_route(self) -> bool:
        """
        Request an obstacle-aware route above the active candidate.

        The candidate supplies horizontal map coordinates. The aircraft
        remains at survey altitude; target depth does not command descent.
        """

        now = time.monotonic()

        if now < self._next_diversion_route_request_at:
            return False

        self._next_diversion_route_request_at = now + self._route_retry_delay

        candidate = self._active_candidate()

        if candidate is None and self._injected_target_location is None:
            rospy.logwarn_throttle(
                2.0,
                "Cannot plan diversion because the active candidate is unavailable",
            )
            return False

        current_position = self._vehicle.current_position()

        if current_position is None:
            rospy.logwarn_throttle(
                2.0,
                "Cannot plan diversion without a fresh vehicle pose",
            )
            return False

        target_x, target_y = self._injected_target_location if self._injected_target_location is not None else (candidate.map_x, candidate.map_y)
        destination = Waypoint(
            x=target_x,
            y=target_y,
            z=self._survey_altitude,
            yaw=0.0,
        )

        if not self._waypoint_is_within_boundary(destination):
            if candidate is not None:
                self._blacklist_active_candidate()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "confirmed candidate lies outside the mission boundary",
            )
            return False

        destination_point = Point(
            x=destination.x,
            y=destination.y,
            z=destination.z,
        )

        target_description = "injected test target" if self._injected_target_location is not None else "candidate {}".format(candidate.candidate_id)
        rospy.loginfo("Planning diversion route above %s at [%.2f, %.2f, %.2f]", target_description, destination.x, destination.y, destination.z)

        route = self._breadcrumb.request_path(
            current_position,
            destination_point,
        )

        if not route:
            rospy.logerr_throttle(
                2.0,
                "No usable Breadcrumb route to %s",
                target_description,
            )
            return False

        self._diversion_route = route
        self._diversion_waypoint_index = 0
        self._diversion_waypoint_active = False
        self._next_diversion_route_request_at = 0.0
        self._publish_visualisation_path(
            self._diversion_route, self._breadcrumb_path_publisher
        )

        rospy.loginfo(
            "Stored candidate diversion route containing %i waypoints",
            len(self._diversion_route),
        )
        return True

    def _send_next_diversion_waypoint(self) -> bool:
        """Submit the next waypoint in the active candidate-diversion route."""

        if self._diversion_waypoint_index >= len(self._diversion_route):
            return False

        if self._vehicle.current_position() is None:
            rospy.logwarn_throttle(
                2.0,
                "Holding diversion because vehicle pose is not fresh",
            )
            return False

        if not self._vehicle.battery_is_ready():
            self.transition_to(
                MissionState.ABORT,
                "battery telemetry is unavailable or unsafe during diversion",
            )
            return False

        waypoint = self._diversion_route[self._diversion_waypoint_index]

        if not self._waypoint_is_within_boundary(waypoint):
            self.transition_to(
                MissionState.ABORT,
                "Breadcrumb produced a diversion waypoint outside the mission boundary",
            )
            return False

        if not self._flight.send_waypoint(waypoint):
            self.transition_to(
                MissionState.ABORT,
                "SPAR rejected a candidate-diversion waypoint",
            )
            return False

        self._diversion_waypoint_index += 1
        self._diversion_waypoint_active = True
        return True

    def _request_centring_route(self) -> bool:
        """Request an obstacle-aware correction above the active candidate."""

        now = time.monotonic()

        if now < self._next_centring_route_request_at:
            return False

        self._next_centring_route_request_at = now + self._route_retry_delay
        candidate = self._active_candidate()
        current_position = self._vehicle.current_position()

        if candidate is None or current_position is None:
            return False

        destination = Waypoint(
            x=candidate.map_x,
            y=candidate.map_y,
            z=self._survey_altitude,
            yaw=0.0,
        )

        if not self._waypoint_is_within_boundary(destination):
            self._blacklist_active_candidate()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "candidate centring position lies outside the mission boundary",
            )
            return False

        destination_point = Point(
            x=destination.x,
            y=destination.y,
            z=destination.z,
        )

        route = self._breadcrumb.request_path(
            current_position,
            destination_point,
        )

        if not route:
            rospy.logerr_throttle(
                2.0,
                "No usable Breadcrumb route for candidate centring",
            )
            return False

        self._centring_route = route
        self._centring_waypoint_index = 0
        self._centring_waypoint_active = False
        self._next_centring_route_request_at = 0.0
        self._publish_visualisation_path(
            self._centring_route, self._breadcrumb_path_publisher
        )
        return True

    def _send_next_centring_waypoint(self) -> bool:
        """Submit the next obstacle-aware candidate-centring waypoint."""

        if self._centring_waypoint_index >= len(self._centring_route):
            return False

        if self._vehicle.current_position() is None:
            return False

        if not self._vehicle.battery_is_ready():
            self.transition_to(
                MissionState.ABORT,
                "battery telemetry is unavailable or unsafe during centring",
            )
            return False

        waypoint = self._centring_route[self._centring_waypoint_index]

        if not self._waypoint_is_within_boundary(waypoint):
            self.transition_to(
                MissionState.ABORT,
                "Breadcrumb produced a centring waypoint outside the mission boundary",
            )
            return False

        if not self._flight.send_waypoint(waypoint):
            self.transition_to(
                MissionState.ABORT,
                "SPAR rejected a candidate-centring waypoint",
            )
            return False

        self._centring_waypoint_index += 1
        self._centring_waypoint_active = True
        return True

    def _handle_search(self):
        """
        Follow the high-level search plan through online Breadcrumb routes.

        Only one Breadcrumb waypoint is active in SPAR at a time. A new
        high-level leg is planned after the previous leg has completed.
        Target-evidence handling may later interrupt this state through the
        explicit SEARCH-to-DIVERT transition.
        """

        candidate = self._strongest_confirmed_candidate()

        if candidate is not None:
            self._select_active_candidate(candidate)

            if self._flight.goal_state() in (
                FlightGoalState.PENDING,
                FlightGoalState.ACTIVE,
            ):
                self._flight.cancel_current_goal()

            # Preserve the high-level search destination index but discard the
            # old Breadcrumb route. RETURN_TO_SEARCH will replan from the
            # aircraft's measured post-investigation position.
            self._active_route = []
            self._route_waypoint_index = 0
            self._next_route_request_at = 0.0
            self.transition_to(
                MissionState.DIVERT,
                "class-specific target evidence passed initial confirmation",
            )
            return

        # No Breadcrumb leg is currently active.
        if not self._active_route:
            if self._search_waypoint_index >= len(
                self._search_waypoints
            ):
                self.transition_to(
                    MissionState.WAIT_FOR_LANDING_SELECTION,
                    "all high-level search destinations completed",
                )
                return

            current_position = self._vehicle.current_position()

            if current_position is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Search progression paused because vehicle pose is stale",
                )
                return

            destination = self._search_waypoints[
                self._search_waypoint_index
            ]

            distance_to_destination = math.sqrt(
                (destination.x - current_position.x) ** 2
                + (destination.y - current_position.y) ** 2
                + (destination.z - current_position.z) ** 2
            )

            # The first search point is the map origin. Dedicated take-off will
            # commonly leave the aircraft already within its tolerance.
            if (
                distance_to_destination
                <= self._search_destination_tolerance
            ):
                rospy.loginfo(
                    "Search destination %i of %i is already reached",
                    self._search_waypoint_index + 1,
                    len(self._search_waypoints),
                )
                self._search_waypoint_index += 1
                return

            self._request_current_search_route()
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            # More intermediate points remain in the active Breadcrumb leg.
            if self._route_waypoint_index < len(
                self._active_route
            ):
                self._send_next_route_waypoint()
                return

            # The final point in this Breadcrumb leg has been reached.
            rospy.loginfo(
                "Completed Breadcrumb route to search destination %i of %i",
                self._search_waypoint_index + 1,
                len(self._search_waypoints),
            )

            self._active_route = []
            self._route_waypoint_index = 0
            self._search_waypoint_index += 1
            return

        if goal_state == FlightGoalState.CANCELLED:
            self.transition_to(
                MissionState.ABORT,
                "search-route waypoint was cancelled",
            )
            return

        self.transition_to(
            MissionState.ABORT,
            "search-route waypoint failed or returned an invalid state",
        )

    def _handle_divert(self):
        """
        Follow an obstacle-aware route above the active candidate.

        Intermittent detections do not immediately cancel the diversion. The
        aircraft investigates the candidate's accumulated map position and
        performs fresh visual confirmation after reaching it.
        """

        if time.monotonic() - self._state_entered_at >= self._diversion_timeout:
            self._suppress_active_candidate_temporarily()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "candidate diversion did not complete before the timeout",
            )
            return

        if self._active_candidate() is None and self._injected_target_location is None:
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "active candidate is no longer available",
            )
            return

        if not self._diversion_route:
            if self._request_diversion_route():
                self._send_next_diversion_waypoint()
            return

        if not self._diversion_waypoint_active:
            if self._diversion_waypoint_index >= len(self._diversion_route):
                self._diversion_route = []
                if self._injected_target_location is not None:
                    self.transition_to(MissionState.DEPLOY, "injected test target reached; explicit test command authorises deployment")
                else:
                    self.transition_to(MissionState.CENTRE, "obstacle-aware diversion route completed")
                return

            self._send_next_diversion_waypoint()
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            self._diversion_waypoint_active = False
            return

        if goal_state == FlightGoalState.CANCELLED:
            self._suppress_active_candidate_temporarily()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "candidate-diversion waypoint was cancelled",
            )
            return

        self.transition_to(
            MissionState.ABORT,
            "candidate-diversion waypoint failed or returned an invalid state",
        )

    def _handle_centre(self):
        """
        Keep the active candidate centred using fresh image observations.

        The image box decides whether correction is required. Candidate map
        coordinates provide the correction destination, while Breadcrumb
        continues to provide obstacle-aware movement.
        """

        now = time.monotonic()
        candidate = self._active_candidate()

        if candidate is None:
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "active candidate is unavailable during centring",
            )
            return

        if now - self._state_entered_at >= self._centre_timeout:
            if self._centring_waypoint_active:
                self._flight.cancel_current_goal()

            self._centring_route = []
            self._centring_waypoint_active = False
            self._candidate_centred_since = None
            self._suppress_active_candidate_temporarily()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "candidate could not be centred before the timeout",
            )
            return

        observation_is_fresh = (
            now - candidate.last_seen_seconds
            <= self._centre_observation_freshness
        )
        within_tolerance = (
            abs(candidate.bbox_center_x_normalized - 0.5)
            <= self._centre_tolerance_x
            and abs(candidate.bbox_center_y_normalized - 0.5)
            <= self._centre_tolerance_y
        )

        if observation_is_fresh and within_tolerance:
            if self._centring_waypoint_active:
                self._flight.cancel_current_goal()
                self._centring_route = []
                self._centring_waypoint_active = False

            if self._active_candidate_is_stably_centred(candidate):
                self._reassessment_baseline_observations = (
                    candidate.observation_count
                )
                self._reassessment_confirmed_since = None
                self.transition_to(
                    MissionState.REASSESS,
                    "candidate remained centred using fresh observations",
                )
            return

        self._candidate_centred_since = None

        # Do not move using an old image observation. Continue hovering while
        # waiting for a fresh detection until the overall centring timeout.
        if not observation_is_fresh:
            return

        if not self._centring_route:
            if self._request_centring_route():
                self._send_next_centring_waypoint()
            return

        if not self._centring_waypoint_active:
            if self._centring_waypoint_index >= len(self._centring_route):
                self._centring_route = []
                return

            self._send_next_centring_waypoint()
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            self._centring_waypoint_active = False
            return

        if goal_state == FlightGoalState.CANCELLED:
            self._centring_route = []
            self._centring_waypoint_active = False
            return

        self.transition_to(
            MissionState.ABORT,
            "candidate-centring waypoint failed or returned an invalid state",
        )

    def _handle_reassess(self):
        """Require fresh post-centring evidence before deployment can proceed."""
        now = time.monotonic()
        candidate = self._active_candidate()

        if not self._vehicle.battery_is_ready():
            self.transition_to(
                MissionState.ABORT,
                "battery telemetry is unavailable or unsafe during reassessment",
            )
            return

        if now - self._state_entered_at >= self._reassessment_timeout:
            self._suppress_active_candidate_temporarily()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "candidate did not pass final confirmation before the timeout",
            )
            return

        if candidate is None:
            return

        observation_is_fresh = (
            now - candidate.last_seen_seconds <= self._centre_observation_freshness
        )
        remains_centred = (
            abs(candidate.bbox_center_x_normalized - 0.5) <= self._centre_tolerance_x
            and abs(candidate.bbox_center_y_normalized - 0.5) <= self._centre_tolerance_y
        )

        if observation_is_fresh and not remains_centred:
            self._candidate_centred_since = None
            self._reassessment_baseline_observations = None
            self.transition_to(
                MissionState.CENTRE,
                "candidate moved outside the image-centre tolerance",
            )
            return

        if not self._active_candidate_passes_final_confirmation(candidate):
            return

        if self._operator.has_fresh_imagery(self._gcs_image_freshness):
            self._veto_request_started_at = None
            self._veto_request_sent = False
            self.transition_to(
                MissionState.WAIT_FOR_VETO,
                "final confirmation passed and Ground Control has fresh imagery",
            )
            return

        self.transition_to(
            MissionState.DEPLOY,
            "final confirmation passed without a useful Ground Control image path",
        )

    def _handle_wait_for_veto(self):
        """Offer a bounded veto while autonomous deployment remains valid."""
        candidate = self._active_candidate()

        if not self._candidate_remains_deployable(candidate):
            self._operator.clear_request()
            self._suppress_active_candidate_temporarily()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "autonomous confirmation was lost while awaiting the optional veto",
            )
            return

        if not self._operator.has_fresh_imagery(self._gcs_image_freshness):
            self._operator.clear_request()
            self.transition_to(
                MissionState.DEPLOY,
                "Ground Control imagery became unavailable; autonomy continues",
            )
            return

        if not self._veto_request_sent:
            self._operator.begin_veto_request(
                candidate.candidate_id,
                candidate.class_id,
                self._operator_veto_timeout,
            )
            self._veto_request_started_at = time.monotonic()
            self._veto_request_sent = True
            return

        decision = self._operator.decision()

        if decision == OperatorDecision.VETO:
            self._blacklist_active_candidate()
            self._operator.clear_request()
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "operator vetoed deployment at the candidate location",
            )
            return

        if decision == OperatorDecision.DEPLOY:
            self._operator.clear_request()
            self.transition_to(
                MissionState.DEPLOY,
                "operator approved deployment",
            )
            return

        if time.monotonic() - self._veto_request_started_at >= self._operator_veto_timeout:
            self._operator.clear_request()
            self.transition_to(
                MissionState.DEPLOY,
                "optional operator veto timed out; autonomy continues",
            )

    def _handle_deploy(self):
        """Descend, deploy the class-mapped payload, then regain survey height."""
        candidate = self._active_candidate()

        if (candidate is None and self._injected_target_location is None) or self._active_candidate_class_id is None:
            self.transition_to(
                MissionState.RETURN_TO_SEARCH,
                "active candidate became unavailable before deployment",
            )
            return

        if not self._vehicle.battery_is_ready():
            self.transition_to(
                MissionState.ABORT,
                "battery telemetry is unavailable or unsafe during deployment",
            )
            return

        if self._deployment_phase == DeploymentPhase.IDLE:
            current_position = self._vehicle.current_position()

            if current_position is None:
                return

            deployment_waypoint = Waypoint(
                x=current_position.x,
                y=current_position.y,
                z=self._deployment_altitude,
                yaw=0.0,
            )

            if not self._waypoint_is_within_boundary(deployment_waypoint):
                self.transition_to(
                    MissionState.ABORT,
                    "deployment position lies outside the mission boundary",
                )
                return

            if not self._flight.send_waypoint(deployment_waypoint):
                self.transition_to(
                    MissionState.ABORT,
                    "SPAR rejected the deployment-altitude waypoint",
                )
                return

            self._deployment_position = (
                deployment_waypoint.x,
                deployment_waypoint.y,
            )
            self._deployment_phase = DeploymentPhase.DESCENDING
            return

        if self._deployment_phase == DeploymentPhase.DESCENDING:
            goal_state = self._flight.goal_state()

            if goal_state in (FlightGoalState.PENDING, FlightGoalState.ACTIVE):
                return

            if goal_state != FlightGoalState.SUCCEEDED:
                self.transition_to(
                    MissionState.ABORT,
                    "aircraft failed to reach the deployment altitude",
                )
                return

            if not self._payload.request_deployment(self._active_candidate_class_id):
                self.transition_to(
                    MissionState.ABORT,
                    "class-appropriate payload could not be commanded",
                )
                return

            self._deployment_commanded_at = time.monotonic()
            self._deployment_phase = DeploymentPhase.WAITING_FOR_PAYLOAD
            return

        if self._deployment_phase == DeploymentPhase.WAITING_FOR_PAYLOAD:
            if self._payload.class_is_deployed(self._active_candidate_class_id):
                ascent_waypoint = Waypoint(
                    x=self._deployment_position[0],
                    y=self._deployment_position[1],
                    z=self._survey_altitude,
                    yaw=0.0,
                )

                if not self._flight.send_waypoint(ascent_waypoint):
                    self.transition_to(
                        MissionState.ABORT,
                        "SPAR rejected the post-deployment ascent waypoint",
                    )
                    return

                self._deployment_phase = DeploymentPhase.ASCENDING
                return

            if time.monotonic() - self._deployment_commanded_at >= self._payload_ack_timeout:
                self.transition_to(
                    MissionState.ABORT,
                    "payload controller did not acknowledge deployment",
                )
            return

        goal_state = self._flight.goal_state()

        if goal_state in (FlightGoalState.PENDING, FlightGoalState.ACTIVE):
            return

        if goal_state != FlightGoalState.SUCCEEDED:
            self.transition_to(
                MissionState.ABORT,
                "aircraft failed to regain survey altitude after deployment",
            )
            return

        self._deployment_phase = DeploymentPhase.IDLE

        if self._payload.all_payloads_deployed():
            self._clear_target_handling()
            self.transition_to(
                MissionState.WAIT_FOR_LANDING_SELECTION,
                "both class-specific payloads were deployed; select the landing ArUco",
            )
            return

        self.transition_to(
            MissionState.RETURN_TO_SEARCH,
            "payload deployed; searching for the remaining target class",
        )

    def _handle_return_to_search(self):
        """Clear target-specific state and replan the interrupted search leg."""
        if self._diversion_waypoint_active or self._centring_waypoint_active:
            self._flight.cancel_current_goal()

        self._clear_target_handling()
        self._active_route = []
        self._route_waypoint_index = 0
        self._next_route_request_at = 0.0
        self.transition_to(
            MissionState.SEARCH,
            "target handling complete; replanning the interrupted search leg",
        )

    def _handle_wait_for_landing_selection(self):
        """Hover while the mission pane requests an observed ArUco ID."""

        self._landing_markers.begin_selection()
        marker = self._landing_markers.selected_marker()
        if marker is None:
            return

        self._selected_landing_marker = marker
        self._landing_route = []
        self._landing_route_index = 0
        self._landing_waypoint_active = False
        self._landing_approach_phase = LandingApproachPhase.PLANNING
        self._landing_descent_command_sent = False
        self._next_landing_route_request_at = 0.0
        self.transition_to(
            MissionState.APPROACH_LANDING,
            "operator selected ArUco {} for landing".format(marker.marker_id),
        )

    def _request_landing_route(self) -> bool:
        """Plan the obstacle-aware survey-height route to the selected ArUco."""

        now = time.monotonic()
        if now < self._next_landing_route_request_at:
            return False
        self._next_landing_route_request_at = now + self._route_retry_delay

        marker = self._selected_landing_marker
        current_position = self._vehicle.current_position()
        if marker is None or current_position is None:
            return False

        destination = Waypoint(x=marker.x, y=marker.y, z=self._survey_altitude, yaw=0.0)
        if not self._waypoint_is_within_boundary(destination):
            self.transition_to(MissionState.ABORT, "selected landing ArUco lies outside the mission boundary")
            return False

        rospy.loginfo(
            "[LAND] Planning Breadcrumb route to ArUco %i at [%.2f, %.2f, %.2f]",
            marker.marker_id, destination.x, destination.y, destination.z,
        )
        route = self._breadcrumb.request_path(
            current_position,
            Point(x=destination.x, y=destination.y, z=destination.z),
        )
        if not route:
            rospy.logerr_throttle(2.0, "[LAND] No safe Breadcrumb route to ArUco %i", marker.marker_id)
            return False

        self._landing_route = route
        self._landing_route_index = 0
        self._landing_waypoint_active = False
        self._landing_approach_phase = LandingApproachPhase.ROUTING
        self._next_landing_route_request_at = 0.0
        self._publish_visualisation_path(self._landing_route, self._breadcrumb_path_publisher)
        return True

    def _send_next_landing_waypoint(self) -> bool:
        """Submit one waypoint from the selected ArUco approach route."""

        if self._landing_route_index >= len(self._landing_route):
            return False
        if self._vehicle.current_position() is None:
            return False
        if not self._vehicle.battery_is_ready():
            self.transition_to(MissionState.ABORT, "battery telemetry is unavailable during landing approach")
            return False

        waypoint = self._landing_route[self._landing_route_index]
        if not self._waypoint_is_within_boundary(waypoint):
            self.transition_to(MissionState.ABORT, "Breadcrumb produced a landing waypoint outside the mission boundary")
            return False
        if not self._flight.send_waypoint(waypoint):
            self.transition_to(MissionState.ABORT, "SPAR rejected an ArUco landing-route waypoint")
            return False

        self._landing_route_index += 1
        self._landing_waypoint_active = True
        return True

    def _handle_approach_landing(self):
        """Route to the selected ArUco, descend above it, then permit LAND."""

        if self._landing_approach_phase == LandingApproachPhase.PLANNING:
            self._request_landing_route()
            return

        if self._landing_approach_phase == LandingApproachPhase.ROUTING:
            if self._landing_waypoint_active:
                goal_state = self._flight.goal_state()
                if goal_state in (FlightGoalState.PENDING, FlightGoalState.ACTIVE):
                    return
                if goal_state != FlightGoalState.SUCCEEDED:
                    self.transition_to(MissionState.ABORT, "failed while travelling to the selected landing ArUco")
                    return
                self._landing_waypoint_active = False

            if self._landing_route_index < len(self._landing_route):
                self._send_next_landing_waypoint()
                return

            self._landing_approach_phase = LandingApproachPhase.DESCENDING

        marker = self._selected_landing_marker
        if marker is None:
            self.transition_to(MissionState.ABORT, "selected landing ArUco became unavailable")
            return

        if not self._landing_descent_command_sent:
            approach = Waypoint(x=marker.x, y=marker.y, z=self._landing_approach_altitude, yaw=0.0)
            if not self._waypoint_is_within_boundary(approach):
                self.transition_to(MissionState.ABORT, "ArUco landing-approach waypoint lies outside the mission boundary")
                return
            if not self._flight.send_waypoint(approach):
                self.transition_to(MissionState.ABORT, "SPAR rejected the ArUco landing-approach waypoint")
                return
            rospy.loginfo("[LAND] Descending to %.2f m above ArUco %i", approach.z, marker.marker_id)
            self._landing_descent_command_sent = True
            return

        goal_state = self._flight.goal_state()
        if goal_state in (FlightGoalState.PENDING, FlightGoalState.ACTIVE):
            return
        if goal_state == FlightGoalState.SUCCEEDED:
            self.transition_to(MissionState.LAND, "ArUco landing approach completed")
            return
        self.transition_to(MissionState.ABORT, "failed to reach the selected ArUco landing altitude")

    def _handle_land(self):
        """
        Submit and supervise a controlled landing after mission completion.

        A normal landing failure transfers authority to ABORT, which may make
        one separate attempt to cancel the existing goal and land in place.
        """

        if not self._landing_command_sent:
            if not self._flight.send_land():
                self.transition_to(
                    MissionState.ABORT,
                    "SPAR rejected the normal landing command",
                )
                return

            self._landing_command_sent = True
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            self.transition_to(
                MissionState.COMPLETE,
                "controlled landing completed",
            )
            return

        self.transition_to(
            MissionState.ABORT,
            "normal landing command failed or was cancelled",
        )

    def _handle_complete(self):
        """
        End the ROS mission node after flight activity has completed.

        COMPLETE is reached only after a successful normal or abort landing.
        """

        if self._completion_handled:
            return

        self._completion_handled = True

        rospy.loginfo(
            "Autonomous mission complete; shutting down mission controller"
        )

        rospy.signal_shutdown(
            "autonomous mission complete"
        )

    def _handle_abort(self):
        """
        Cancel the active flight goal and request one landing-in-place attempt.

        ABORT does not repeatedly issue landing commands. If SPAR cannot accept
        or complete the abort landing, the controller remains in ABORT and
        reports that operator intervention is required.
        """

        if not self._abort_landing_attempted:
            self._abort_landing_attempted = True

            self._flight.cancel_current_goal()

            self._abort_landing_submitted = (
                self._flight.send_land()
            )

            if not self._abort_landing_submitted:
                rospy.logerr(
                    "Abort landing could not be submitted; "
                    "operator intervention is required"
                )
                return

            rospy.logwarn(
                "Mission aborted; landing in place"
            )
            return

        if not self._abort_landing_submitted:
            rospy.logerr_throttle(
                2.0,
                "Abort landing is unavailable; "
                "operator intervention is required",
            )
            return

        goal_state = self._flight.goal_state()

        if goal_state in (
            FlightGoalState.PENDING,
            FlightGoalState.ACTIVE,
        ):
            return

        if goal_state == FlightGoalState.SUCCEEDED:
            self.transition_to(
                MissionState.COMPLETE,
                "abort landing completed",
            )
            return

        rospy.logerr_throttle(
            2.0,
            "Abort landing failed or was cancelled; "
            "operator intervention is required",
        )

    def shutdown(self):
        """
        Release mission control during ROS shutdown.

        An externally interrupted mission cancels its current SPAR goal.
        A successfully completed mission has already finished landing and
        therefore does not need its completed goal cancelled.
        """

        if self.state != MissionState.COMPLETE:
            self._flight.cancel_current_goal()

        rospy.loginfo(
            "Autonomous mission controller stopped in state: %s",
            self.state.name,
        )

    def run(self):
        """
        Complete mission preflight, start periodic state processing and keep
        the ROS node alive until shutdown.
        """

        rospy.on_shutdown(self.shutdown)

        if not self.perform_preflight():
            rospy.logerr(
                "Autonomous mission preflight failed; state: %s",
                self.state.name,
            )
            rospy.spin()
            return

        self.transition_to(
            MissionState.TAKEOFF,
            "all autonomous mission preflight checks passed",
        )

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._update_rate),
            self._update,
        )

        rospy.loginfo(
            "Autonomous mission controller running at %.1f Hz",
            self._update_rate,
        )

        rospy.spin()
