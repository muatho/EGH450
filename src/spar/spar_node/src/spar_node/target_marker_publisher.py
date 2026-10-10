"""
Publish the mission's remembered person and backpack candidates for RViz.

This component is visualisation-only. It does not accumulate evidence, confirm
targets, select diversions or authorise deployment. Mission decisions remain
inside MissionController and TargetEvidenceAccumulator.

A single MarkerArray topic carries every target. ROS marker namespaces allow
RViz to distinguish and independently show or hide people, backpacks and text
labels without requiring separate topics.

There is no direct legacy predecessor. The legacy mission consumed detections
but did not publish its remembered person/backpack locations as RViz markers.
"""

import time

import rospy

from visualization_msgs.msg import Marker, MarkerArray


class TargetMarkerPublisher:
    """Convert target-evidence snapshots into map-frame RViz markers."""

    PERSON_CLASS_ID = 0
    BACKPACK_CLASS_ID = 1

    # Injected targets use IDs well above the normal evidence-candidate range.
    # This prevents a test marker from replacing a genuine candidate marker
    # that happens to belong to the same marker namespace.
    INJECTED_MARKER_ID_BASE = 1000000

    def __init__(self):
        """
        Create one latched publisher for the complete target-marker display.

        Latching allows RViz to receive the most recent world-model display
        immediately after RViz starts, rather than waiting for another target
        observation.
        """

        self._topic = rospy.get_param(
            "~target_marker_topic",
            "/world_model/target_markers",
        )

        self._publisher = rospy.Publisher(
            self._topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        rospy.loginfo(
            "Target markers publishing on %s",
            self._topic,
        )

    def publish(
        self,
        candidates,
        injected_class_id=None,
        injected_location=None,
    ):
        """
        Publish all remembered detector candidates and any injected test target.

        Candidate evidence remains authoritative. This method only chooses how
        that evidence is represented visually.

        Fresh candidates are drawn strongly. Candidates that have not received
        support recently remain visible but faded, showing the distinction
        between a current detection and a remembered location.

        Args:
            candidates: Immutable EvidenceCandidate snapshots.
            injected_class_id: Class of the active explicit test target.
            injected_location: Optional test-target map coordinate `(x, y)`.
        """

        marker_array = MarkerArray()

        # Each publication describes the complete target display. Clearing the
        # preceding display first ensures that a candidate removed by a future
        # reset command also disappears from RViz rather than remaining as a
        # stale marker indefinitely.
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        now = time.monotonic()

        for candidate in candidates:
            candidate_age = now - candidate.last_seen_seconds

            # One second is also the current mission freshness limit for
            # initial target confirmation. The faded state therefore indicates
            # that the mission remembers the candidate but cannot presently
            # use it to begin a diversion.
            alpha = 0.9 if candidate_age <= 1.0 else 0.25

            marker_array.markers.append(
                self._create_body_marker(
                    marker_id=candidate.candidate_id,
                    class_id=candidate.class_id,
                    x=candidate.map_x,
                    y=candidate.map_y,
                    alpha=alpha,
                )
            )

            marker_array.markers.append(
                self._create_text_marker(
                    marker_id=candidate.candidate_id,
                    class_id=candidate.class_id,
                    x=candidate.map_x,
                    y=candidate.map_y,
                    score=candidate.score,
                    alpha=alpha,
                )
            )

        if (
            injected_class_id is not None
            and injected_location is not None
        ):
            # The explicit terminal injection bypasses detector evidence, so it
            # has no EvidenceCandidate record. Display it separately using the
            # same class colour and shape as a genuine candidate.
            injected_marker_id = (
                self.INJECTED_MARKER_ID_BASE
                + injected_class_id
            )

            marker_array.markers.append(
                self._create_body_marker(
                    marker_id=injected_marker_id,
                    class_id=injected_class_id,
                    x=injected_location[0],
                    y=injected_location[1],
                    alpha=1.0,
                )
            )

            marker_array.markers.append(
                self._create_text_marker(
                    marker_id=injected_marker_id,
                    class_id=injected_class_id,
                    x=injected_location[0],
                    y=injected_location[1],
                    score=None,
                    alpha=1.0,
                    injected=True,
                )
            )

        self._publisher.publish(marker_array)

    def _create_body_marker(
        self,
        marker_id,
        class_id,
        x,
        y,
        alpha,
    ):
        """
        Create the geometric RViz symbol for one target.

        The cylinder and cube are symbolic—not estimated physical object
        dimensions. Their fixed heights keep them visible above the map while
        the estimated target X/Y remains the meaningful navigation information.
        """

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.color.a = alpha

        if class_id == self.PERSON_CLASS_ID:
            # People use a blue cylinder. Blue deliberately avoids the red
            # convention already used by the ArUco landing markers.
            marker.ns = "target_people"
            marker.type = Marker.CYLINDER
            marker.pose.position.z = 0.85
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 1.7
            marker.color.r = 0.1
            marker.color.g = 0.45
            marker.color.b = 1.0

        elif class_id == self.BACKPACK_CLASS_ID:
            # Backpacks use a yellow cube. The distinct colour and geometry
            # make the classes distinguishable even when colour is difficult
            # to see on a remote or low-quality Ground Control display.
            marker.ns = "target_backpacks"
            marker.type = Marker.CUBE
            marker.pose.position.z = 0.2
            marker.scale.x = 0.45
            marker.scale.y = 0.35
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.1

        else:
            # Unsupported classes should already have been rejected by the
            # observation buffer. Keeping this guard prevents an accidental
            # unknown class from appearing as a valid known target in RViz.
            raise ValueError(
                "Unsupported target marker class: {}".format(
                    class_id
                )
            )

        return marker

    def _create_text_marker(
        self,
        marker_id,
        class_id,
        x,
        y,
        score,
        alpha,
        injected=False,
    ):
        """Create a camera-facing class and evidence label above one target."""

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"

        # Labels have a separate namespace from the geometric bodies, so using
        # the same ID cannot replace the body marker.
        marker.ns = "target_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = alpha

        if class_id == self.PERSON_CLASS_ID:
            class_name = "Person"
            marker.pose.position.z = 1.9
        elif class_id == self.BACKPACK_CLASS_ID:
            class_name = "Backpack"
            marker.pose.position.z = 0.65
        else:
            raise ValueError(
                "Unsupported target label class: {}".format(
                    class_id
                )
            )

        if injected:
            # The wording distinguishes a deliberate mission test from a
            # candidate produced by the real detector/evidence path.
            marker.text = "{} test target".format(
                class_name
            )
        else:
            marker.text = "{} {} | score {:.2f}".format(
                class_name,
                marker_id,
                score,
            )

        return marker