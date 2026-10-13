#!/usr/bin/env python3

"""Confirm and remember map-frame person and backpack observations."""

from collections import deque
from math import hypot, isfinite
from statistics import median

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from depthai_publisher.msg import TargetObservationArray


class TargetWorldModel:
    """Publish current confirmed targets and persistent RViz markers."""

    PERSON_CLASS_ID = 0
    BACKPACK_CLASS_ID = 1

    # A target must appear in at least three of the five most recent frames.
    REQUIRED_FRAMES = 3
    WINDOW_SIZE = 5

    # Observations within this XY distance may represent the same object.
    MATCH_RADIUS = 0.30

    CLASS_NAMES = {
        PERSON_CLASS_ID: "Person",
        BACKPACK_CLASS_ID: "Backpack",
    }

    def __init__(self):
        """Create the independent target-memory inputs and outputs."""

        self.frames = deque(maxlen=self.WINDOW_SIZE)

        # Both initial diversion and final confirmation load these values from
        # the shared target-confidence configuration.
        self.minimum_confidence = {
            self.PERSON_CLASS_ID: float(rospy.get_param(
                "/target_policy/person_confidence_threshold"
            )),
            self.BACKPACK_CLASS_ID: float(rospy.get_param(
                "/target_policy/backpack_confidence_threshold"
            )),
        }
        for class_id, threshold in self.minimum_confidence.items():
            if not 0.0 <= threshold <= 1.0:
                raise ValueError(
                    "{} confidence threshold must be between 0 and 1".format(
                        self.CLASS_NAMES[class_id]
                    )
                )

        # Coverage is the percentage of the source camera frame occupied by
        # one detection box. A zero threshold disables the size gate.
        self.minimum_coverage_percent = {
            self.PERSON_CLASS_ID: float(rospy.get_param(
                "/target_policy/person_min_coverage_percent"
            )),
            self.BACKPACK_CLASS_ID: float(rospy.get_param(
                "/target_policy/backpack_min_coverage_percent"
            )),
        }
        for class_id, threshold in self.minimum_coverage_percent.items():
            if not 0.0 <= threshold <= 100.0:
                raise ValueError(
                    "{} minimum coverage must be between 0 and 100 percent".format(
                        self.CLASS_NAMES[class_id]
                    )
                )

        # Confirmed targets remain in the RViz world model after disappearing.
        self.remembered_targets = {
            self.PERSON_CLASS_ID: [],
            self.BACKPACK_CLASS_ID: [],
        }
        # Manually rejected locations stay hidden for the rest of this mission.
        self.rejected_targets = {
            self.PERSON_CLASS_ID: [],
            self.BACKPACK_CLASS_ID: [],
        }
        self.next_marker_id = 1

        # A new mission clears targets retained from the previous run.
        self.clear_service = rospy.Service(
            "/world_model/clear_targets",
            Trigger,
            self.clear_targets,
        )

        # This topic contains only targets currently satisfying 3-of-5.
        self.confirmed_target_publisher = rospy.Publisher(
            "/objects/confirmed_targets",
            Detection3DArray,
            queue_size=1,
        )

        # This latched topic contains every remembered confirmed location.
        self.marker_publisher = rospy.Publisher(
            "/world_model/target_markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.observation_subscriber = rospy.Subscriber(
            "/objects/target_observations",
            TargetObservationArray,
            self.observation_callback,
            queue_size=1,
        )

        # Explicit test targets use the same persistent RViz display without
        # being inserted into the detector's live confirmation output.
        self.injected_target_subscriber = rospy.Subscriber(
            "/objects/injected_test_target",
            Detection3DArray,
            self.injected_target_callback,
            queue_size=1,
        )

        self.rejected_target_subscriber = rospy.Subscriber(
            "/objects/rejected_target",
            Detection3D,
            self.rejected_target_callback,
            queue_size=10,
        )

        self.locked_target_subscriber = rospy.Subscriber(
            "/objects/locked_target",
            Detection3D,
            self.locked_target_callback,
            queue_size=10,
        )

        # Persistent world markers do not require camera-frame update rates.
        self.marker_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.publish_markers,
        )

        rospy.loginfo(
            "Target world model: /objects/target_observations -> "
            "/objects/confirmed_targets and /world_model/target_markers; "
            "test input: /objects/injected_test_target"
        )
        rospy.loginfo(
            "Diversion minimum coverage: person %.3f%%, backpack %.3f%%",
            self.minimum_coverage_percent[self.PERSON_CLASS_ID],
            self.minimum_coverage_percent[self.BACKPACK_CLASS_ID],
        )

    # -----------------------------------------------------------------
    # MISSION RESET
    # -----------------------------------------------------------------

    def clear_targets(self, request):
        """Clear target evidence, memory and persistent RViz markers."""

        del request
        cleared_count = sum(
            len(targets) for targets in self.remembered_targets.values()
        )
        self.frames.clear()
        self.remembered_targets = {
            self.PERSON_CLASS_ID: [],
            self.BACKPACK_CLASS_ID: [],
        }
        self.rejected_targets = {
            self.PERSON_CLASS_ID: [],
            self.BACKPACK_CLASS_ID: [],
        }
        self.next_marker_id = 1

        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

        self.publish_confirmed_targets(rospy.Time.now(), [])
        rospy.loginfo("Cleared %d remembered target(s)", cleared_count)
        return TriggerResponse(
            success=True,
            message="Cleared {} target(s)".format(cleared_count),
        )

    # -----------------------------------------------------------------
    # LIVE DETECTION CONFIRMATION
    # -----------------------------------------------------------------

    def observation_callback(self, message):
        """Process one complete camera frame of target observations."""

        accepted_observations = []

        if message.map_frame_id not in ("map", "/map"):
            rospy.logwarn_throttle(
                5.0,
                "Target world model expected map coordinates, received '%s'",
                message.map_frame_id,
            )
        else:
            for observation in message.observations:
                minimum_confidence = self.minimum_confidence.get(
                    observation.class_id
                )

                if minimum_confidence is None:
                    continue
                if observation.confidence < minimum_confidence:
                    continue

                coverage_percent = self.bounding_box_coverage_percent(
                    observation,
                    message.image_width,
                    message.image_height,
                )
                if coverage_percent < self.minimum_coverage_percent[
                    observation.class_id
                ]:
                    continue

                if not observation.map_position_valid:
                    continue

                position = observation.map_position

                if not (
                    isfinite(position.x)
                    and isfinite(position.y)
                    and isfinite(position.z)
                ):
                    continue

                accepted_observations.append(observation)

        # Empty frames are retained because missing detections are evidence too.
        self.frames.append(accepted_observations)

        confirmed_targets = self.find_confirmed_targets()
        self.update_remembered_targets(confirmed_targets)

        self.publish_confirmed_targets(
            message.header.stamp,
            confirmed_targets,
        )

    @staticmethod
    def bounding_box_coverage_percent(
        observation,
        image_width,
        image_height,
    ):
        """Return box area as a percentage of the source camera frame."""

        if image_width == 0 or image_height == 0:
            return 0.0

        # Box dimensions and image dimensions are both in source-image pixels;
        # the detector publisher has already removed resize/letterbox padding.
        box_width = min(max(observation.bbox_width, 0.0), image_width)
        box_height = min(max(observation.bbox_height, 0.0), image_height)
        return 100.0 * box_width * box_height / (
            float(image_width) * float(image_height)
        )

    # -----------------------------------------------------------------
    # TEST, REJECTION AND POSITION-LOCK UPDATES
    # -----------------------------------------------------------------

    def injected_target_callback(self, message):
        """Remember explicit test targets for RViz display."""

        if message.header.frame_id not in ("map", "/map"):
            rospy.logwarn(
                "Ignoring injected target in frame '%s'; expected map",
                message.header.frame_id,
            )
            return

        injected_targets = []

        for detection in message.detections:
            if not detection.results:
                continue

            result = max(
                detection.results,
                key=lambda hypothesis: hypothesis.score,
            )
            class_id = int(result.id)
            position = result.pose.pose.position

            if class_id not in self.CLASS_NAMES:
                continue
            if not (
                isfinite(position.x)
                and isfinite(position.y)
                and isfinite(position.z)
            ):
                continue

            injected_targets.append({
                "class_id": class_id,
                "confidence": result.score,
                "position": [position.x, position.y, position.z],
            })

        self.update_remembered_targets(
            injected_targets,
            source="test",
        )

    def rejected_target_callback(self, message):
        """Remove and suppress one rejected RViz target location."""

        if message.header.frame_id not in ("map", "/map"):
            rospy.logwarn(
                "Ignoring rejected target in frame '%s'; expected map",
                message.header.frame_id,
            )
            return
        if not message.results:
            return

        result = max(
            message.results,
            key=lambda hypothesis: hypothesis.score,
        )
        class_id = int(result.id)
        position_message = result.pose.pose.position
        position = [
            position_message.x,
            position_message.y,
            position_message.z,
        ]

        if class_id not in self.CLASS_NAMES:
            return
        if not all(isfinite(value) for value in position):
            return

        if not any(
            self.position_distance(existing, position) <= self.MATCH_RADIUS
            for existing in self.rejected_targets[class_id]
        ):
            self.rejected_targets[class_id].append(position)

        removed_marker_ids = []
        retained_targets = []

        for target in self.remembered_targets[class_id]:
            if self.position_distance(
                target["position"],
                position,
            ) <= self.MATCH_RADIUS:
                removed_marker_ids.append(target["id"])
            else:
                retained_targets.append(target)

        self.remembered_targets[class_id] = retained_targets
        self.delete_markers(removed_marker_ids)

        rospy.loginfo(
            "Removed rejected %s location from the RViz world model",
            self.CLASS_NAMES[class_id],
        )

    def locked_target_callback(self, message):
        """Freeze one target marker at the location selected by the mission."""

        if message.header.frame_id not in ("map", "/map") or not message.results:
            return

        result = max(
            message.results,
            key=lambda hypothesis: hypothesis.score,
        )
        class_id = int(result.id)
        position_message = result.pose.pose.position
        position = [
            position_message.x,
            position_message.y,
            position_message.z,
        ]

        if class_id not in self.CLASS_NAMES:
            return
        if not all(isfinite(value) for value in position):
            return

        stored_targets = self.remembered_targets[class_id]
        nearby_targets = [
            target
            for target in stored_targets
            if self.position_distance(
                target["position"],
                position,
            ) <= self.MATCH_RADIUS
        ]

        if nearby_targets:
            target = min(
                nearby_targets,
                key=lambda candidate: self.position_distance(
                    candidate["position"],
                    position,
                ),
            )
            target["position"] = list(position)
            target["confidence"] = result.score
            target["locked"] = True
        else:
            stored_targets.append({
                "id": self.next_marker_id,
                "position": list(position),
                "confidence": result.score,
                "source": "detector",
                "locked": True,
            })
            self.next_marker_id += 1

    def delete_markers(self, marker_ids):
        """Delete selected target shapes and labels from RViz."""

        if not marker_ids:
            return

        message = MarkerArray()
        stamp = rospy.Time.now()

        for marker_id in marker_ids:
            for namespace in ("target_locations", "target_labels"):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = "map"
                marker.ns = namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                message.markers.append(marker)

        self.marker_publisher.publish(message)

    # -----------------------------------------------------------------
    # SPATIAL ASSOCIATION AND PERSISTENT TARGET MEMORY
    # -----------------------------------------------------------------

    @staticmethod
    def observation_distance(first, second):
        """Return the XY distance between two target observations."""

        return hypot(
            first.map_position.x - second.map_position.x,
            first.map_position.y - second.map_position.y,
        )

    @staticmethod
    def position_distance(first, second):
        """Return the XY distance between two stored positions."""

        return hypot(
            first[0] - second[0],
            first[1] - second[1],
        )

    def find_confirmed_targets(self):
        """Return every spatial target appearing in at least 3 of 5 frames."""

        confirmed_targets = []

        for frame in self.frames:
            for anchor in frame:
                supporting_observations = []

                # At most one nearby observation from each frame may support
                # this candidate.
                for comparison_frame in self.frames:
                    matches = [
                        observation
                        for observation in comparison_frame
                        if (
                            observation.class_id == anchor.class_id
                            and self.observation_distance(
                                anchor,
                                observation,
                            ) <= self.MATCH_RADIUS
                        )
                    ]

                    if matches:
                        supporting_observations.append(
                            min(
                                matches,
                                key=lambda observation:
                                    self.observation_distance(
                                        anchor,
                                        observation,
                                    ),
                            )
                        )

                if len(supporting_observations) < self.REQUIRED_FRAMES:
                    continue

                candidate = {
                    "class_id": anchor.class_id,
                    "confidence": median([
                        observation.confidence
                        for observation in supporting_observations
                    ]),
                    "position": [
                        median([
                            observation.map_position.x
                            for observation in supporting_observations
                        ]),
                        median([
                            observation.map_position.y
                            for observation in supporting_observations
                        ]),
                        median([
                            observation.map_position.z
                            for observation in supporting_observations
                        ]),
                    ],
                    "supporting_frames": len(supporting_observations),
                }

                matching_candidates = [
                    existing
                    for existing in confirmed_targets
                    if (
                        existing["class_id"] == candidate["class_id"]
                        and self.position_distance(
                            existing["position"],
                            candidate["position"],
                        ) <= self.MATCH_RADIUS
                    )
                ]

                if not matching_candidates:
                    confirmed_targets.append(candidate)
                    continue

                existing = matching_candidates[0]
                existing_rank = (
                    existing["supporting_frames"],
                    existing["confidence"],
                )
                candidate_rank = (
                    candidate["supporting_frames"],
                    candidate["confidence"],
                )

                if candidate_rank > existing_rank:
                    confirmed_targets[
                        confirmed_targets.index(existing)
                    ] = candidate

        return confirmed_targets

    def update_remembered_targets(self, confirmed_targets, source="detector"):
        """Add new confirmed objects or update their remembered locations."""

        for target in confirmed_targets:
            if any(
                self.position_distance(
                    rejected_position,
                    target["position"],
                ) <= self.MATCH_RADIUS
                for rejected_position in self.rejected_targets[
                    target["class_id"]
                ]
            ):
                continue

            stored_targets = self.remembered_targets[
                target["class_id"]
            ]

            nearby_targets = [
                stored_target
                for stored_target in stored_targets
                if self.position_distance(
                    stored_target["position"],
                    target["position"],
                ) <= self.MATCH_RADIUS
            ]

            if nearby_targets:
                stored_target = min(
                    nearby_targets,
                    key=lambda candidate: self.position_distance(
                        candidate["position"],
                        target["position"],
                    ),
                )
                if not stored_target.get("locked", False):
                    stored_target["position"] = list(target["position"])
                stored_target["confidence"] = target["confidence"]
                if source == "test":
                    stored_target["source"] = "test"
                continue

            rospy.loginfo(
                "[%s] %s at (%.2f, %.2f, %.2f)",
                "TEST TARGET" if source == "test" else "TARGET CONFIRMED",
                self.CLASS_NAMES[target["class_id"]],
                target["position"][0],
                target["position"][1],
                target["position"][2],
            )

            stored_targets.append({
                "id": self.next_marker_id,
                "position": list(target["position"]),
                "confidence": target["confidence"],
                "source": source,
                "locked": False,
            })

            self.next_marker_id += 1

    # -----------------------------------------------------------------
    # RVIZ AND CONFIRMED-TARGET OUTPUT
    # -----------------------------------------------------------------

    def publish_markers(self, timer_event):
        """Publish remembered target locations and readable map labels."""

        del timer_event
        message = MarkerArray()
        stamp = rospy.Time.now()

        for class_id, targets in self.remembered_targets.items():
            for target in targets:
                position = target["position"]

                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = "map"
                marker.ns = "target_locations"
                marker.id = target["id"]
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = position[2]
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                if class_id == self.PERSON_CLASS_ID:
                    # A blue cube distinguishes a person in the map.
                    marker.scale.x = 0.35
                    marker.scale.y = 0.35
                    marker.scale.z = 0.35
                    marker.color.b = 1.0
                else:
                    # A compact yellow cube distinguishes a backpack.
                    marker.scale.x = 0.35
                    marker.scale.y = 0.35
                    marker.scale.z = 0.35
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                message.markers.append(marker)

                label = Marker()
                label.header = marker.header
                label.ns = "target_labels"
                label.id = target["id"]
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = position[0]
                label.pose.position.y = position[1]
                label.pose.position.z = position[2] + 0.30
                label.pose.orientation.w = 1.0
                label.scale.z = 0.16
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                label_name = self.CLASS_NAMES[class_id]
                if target.get("source") == "test":
                    label_name = "TEST {}".format(label_name)
                label.text = "{} | X: {:.2f} Y: {:.2f} Z: {:.2f}".format(
                    label_name,
                    position[0],
                    position[1],
                    position[2],
                )
                message.markers.append(label)

        self.marker_publisher.publish(message)

    def publish_confirmed_targets(self, stamp, confirmed_targets):
        """Publish targets that currently satisfy the 3-of-5 requirement."""

        message = Detection3DArray()
        message.header.stamp = stamp
        message.header.frame_id = "map"

        for target in confirmed_targets:
            detection = Detection3D()
            detection.header = message.header

            result = ObjectHypothesisWithPose()
            result.id = target["class_id"]
            result.score = target["confidence"]
            result.pose.pose.position.x = target["position"][0]
            result.pose.pose.position.y = target["position"][1]
            result.pose.pose.position.z = target["position"][2]
            result.pose.pose.orientation.w = 1.0
            detection.results.append(result)

            detection.bbox.center.position.x = target["position"][0]
            detection.bbox.center.position.y = target["position"][1]
            detection.bbox.center.position.z = target["position"][2]
            detection.bbox.center.orientation.w = 1.0

            message.detections.append(detection)

        self.confirmed_target_publisher.publish(message)


def main():
    """Run the target world-model node."""

    rospy.init_node("target_world_model")
    TargetWorldModel()
    rospy.spin()


if __name__ == "__main__":
    main()
