"""
Thread-safe input boundary for raw person and backpack observations.

This module validates and stores per-frame facts from the DepthAI pipeline.
It does not track objects, accumulate evidence or make mission decisions.
"""

import math
import threading
import time

from dataclasses import dataclass
from typing import Optional, Tuple

import rospy

from depthai_publisher.msg import TargetObservationArray


@dataclass(frozen=True)
class RawTargetObservation:
    """Validated facts from one untracked YOLO observation."""

    class_id: int
    confidence: float
    bbox_center_x: float
    bbox_center_y: float
    bbox_width: float
    bbox_height: float
    image_width: int
    image_height: int
    map_x: float
    map_y: float
    map_z: float
    depth_median_metres: float
    depth_mad_metres: float
    valid_depth_samples: int
    valid_depth_fraction: float

    @property
    def box_coverage(self) -> float:
        """Return the fraction of the complete image covered by the box."""
        return (
            self.bbox_width * self.bbox_height
            / float(self.image_width * self.image_height)
        )


@dataclass(frozen=True)
class ObservationFrame:
    """Validated raw observations produced from one source image."""

    stamp_seconds: float
    sequence: int
    frame_id: str
    received_at: float
    observations: Tuple[RawTargetObservation, ...]


class TargetObservationBuffer:
    """Retain the latest complete frame of raw target observations."""

    PERSON_CLASS_ID = 0
    BACKPACK_CLASS_ID = 1
    SUPPORTED_CLASS_IDS = {
        PERSON_CLASS_ID,
        BACKPACK_CLASS_ID,
    }

    def __init__(self):
        self._lock = threading.RLock()
        self._latest_frame = None

        self._topic = rospy.get_param(
            "~target_observation_topic",
            "/objects/target_observations",
        )

        self._subscriber = rospy.Subscriber(
            self._topic,
            TargetObservationArray,
            self._observation_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Target observation buffer listening on %s",
            self._topic,
        )

    def _observation_callback(self, message: TargetObservationArray):
        """
        Validate and atomically store one complete detector frame.

        Empty arrays are retained because a processed frame containing no
        targets is meaningful input for later evidence decay and flicker
        analysis.
        """

        observations = []

        for message_observation in message.observations:
            observation = self._convert_observation(
                message_observation
            )

            if observation is not None:
                observations.append(observation)

        frame = ObservationFrame(
            stamp_seconds=message.header.stamp.to_sec(),
            sequence=int(message.header.seq),
            frame_id=message.header.frame_id,
            received_at=time.monotonic(),
            observations=tuple(observations),
        )

        with self._lock:
            self._latest_frame = frame

    def _convert_observation(self, observation):
        """Return one validated immutable observation or None."""

        scalar_values = (
            observation.confidence,
            observation.bbox_center_x,
            observation.bbox_center_y,
            observation.bbox_width,
            observation.bbox_height,
            observation.map_position.x,
            observation.map_position.y,
            observation.map_position.z,
            observation.depth_median_metres,
            observation.depth_mad_metres,
            observation.valid_depth_fraction,
        )

        if not all(math.isfinite(value) for value in scalar_values):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation containing non-finite values",
            )
            return None

        if observation.class_id not in self.SUPPORTED_CLASS_IDS:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring unsupported target class: %i",
                observation.class_id,
            )
            return None

        if not 0.0 <= observation.confidence <= 1.0:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation with invalid confidence",
            )
            return None

        if observation.image_width <= 0 or observation.image_height <= 0:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation with invalid image dimensions",
            )
            return None

        if observation.bbox_width <= 0 or observation.bbox_height <= 0:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation with invalid bounding box",
            )
            return None

        if (
            observation.bbox_width > observation.image_width
            or observation.bbox_height > observation.image_height
            or not 0.0 <= observation.bbox_center_x <= observation.image_width
            or not 0.0 <= observation.bbox_center_y <= observation.image_height
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation outside its image dimensions",
            )
            return None

        if (
            observation.depth_median_metres <= 0
            or observation.depth_mad_metres < 0
            or observation.valid_depth_samples <= 0
            or not 0.0 <= observation.valid_depth_fraction <= 1.0
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring target observation with invalid depth evidence",
            )
            return None

        return RawTargetObservation(
            class_id=int(observation.class_id),
            confidence=float(observation.confidence),
            bbox_center_x=float(observation.bbox_center_x),
            bbox_center_y=float(observation.bbox_center_y),
            bbox_width=float(observation.bbox_width),
            bbox_height=float(observation.bbox_height),
            image_width=int(observation.image_width),
            image_height=int(observation.image_height),
            map_x=float(observation.map_position.x),
            map_y=float(observation.map_position.y),
            map_z=float(observation.map_position.z),
            depth_median_metres=float(observation.depth_median_metres),
            depth_mad_metres=float(observation.depth_mad_metres),
            valid_depth_samples=int(observation.valid_depth_samples),
            valid_depth_fraction=float(observation.valid_depth_fraction),
        )

    def latest_frame(self) -> Optional[ObservationFrame]:
        """
        Return the latest immutable detector frame.

        None means no TargetObservationArray has arrived. A returned frame may
        legitimately contain an empty observations tuple.
        """

        with self._lock:
            return self._latest_frame

    def latest_frame_age(self) -> Optional[float]:
        """Return seconds since the latest observation array was received."""

        with self._lock:
            if self._latest_frame is None:
                return None

            return time.monotonic() - self._latest_frame.received_at