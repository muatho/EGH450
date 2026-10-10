"""
Spatial and temporal evidence for raw target observations.

This module keeps person and backpack evidence separate. It may associate
nearby observations with the same spatial candidate, but it does not provide
persistent tracked identity or make navigation decisions.
"""

import math
import threading

from dataclasses import dataclass
from typing import Dict, Mapping, Tuple

from spar_node.target_observations import ObservationFrame, RawTargetObservation


@dataclass(frozen=True)
class EvidenceCandidate:
    """Read-only summary of one spatial target candidate."""

    candidate_id: int
    class_id: int
    map_x: float
    map_y: float
    map_z: float
    score: float
    observation_count: int
    last_seen_seconds: float
    latest_confidence: float
    bbox_center_x_normalized: float
    bbox_center_y_normalized: float
    box_coverage: float


@dataclass
class _EvidenceCluster:
    """Mutable internal state for nearby observations of one class."""

    candidate_id: int
    class_id: int
    map_x: float
    map_y: float
    map_z: float
    score: float
    observation_count: int
    last_seen_seconds: float
    last_updated_seconds: float
    last_contribution_seconds: float
    latest_observation: RawTargetObservation


class TargetEvidenceAccumulator:
    """
    Accumulate decaying evidence in separate class-specific spatial clusters.

    All behaviour-affecting values are injected explicitly. This prevents
    provisional assumptions from being hidden inside the evidence algorithm.
    """

    COVERAGE_BAND_COUNT = 20

    def __init__(
        self,
        cluster_radius: float,
        evidence_half_life: float,
        minimum_observation_interval: float,
        maximum_observation_contribution: float,
        coverage_weights: Mapping[int, Tuple[float, ...]],
    ):
        parameters = {
            "cluster_radius": cluster_radius,
            "evidence_half_life": evidence_half_life,
            "minimum_observation_interval": minimum_observation_interval,
            "maximum_observation_contribution": maximum_observation_contribution,
        }

        for name, value in parameters.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "{} must be finite and greater than zero".format(name)
                )

        validated_weights = {}

        for class_id, class_weights in coverage_weights.items():
            if len(class_weights) != self.COVERAGE_BAND_COUNT:
                raise ValueError(
                    "Each class requires exactly 20 coverage-band weights"
                )

            converted_weights = tuple(float(weight) for weight in class_weights)

            if not all(
                math.isfinite(weight) and weight >= 0
                for weight in converted_weights
            ):
                raise ValueError(
                    "Coverage weights must be finite and non-negative"
                )

            validated_weights[int(class_id)] = converted_weights

        if not validated_weights:
            raise ValueError(
                "At least one class-specific coverage-weight table is required"
            )

        self._cluster_radius = float(cluster_radius)
        self._evidence_half_life = float(evidence_half_life)
        self._minimum_observation_interval = float(
            minimum_observation_interval
        )
        self._maximum_observation_contribution = float(
            maximum_observation_contribution
        )
        self._coverage_weights = validated_weights

        self._lock = threading.RLock()
        self._clusters: Dict[int, Dict[int, _EvidenceCluster]] = {
            class_id: {} for class_id in validated_weights
        }
        self._next_candidate_id = 1
        self._last_processed_frame = None
        self._latest_processed_time = None

    def candidates(self) -> Tuple[EvidenceCandidate, ...]:
        """Return immutable summaries of all current candidate clusters."""

        with self._lock:
            candidates = []

            for class_clusters in self._clusters.values():
                for cluster in class_clusters.values():
                    candidates.append(
                        EvidenceCandidate(
                            candidate_id=cluster.candidate_id,
                            class_id=cluster.class_id,
                            map_x=cluster.map_x,
                            map_y=cluster.map_y,
                            map_z=cluster.map_z,
                            score=cluster.score,
                            observation_count=cluster.observation_count,
                            last_seen_seconds=cluster.last_seen_seconds,
                            latest_confidence=cluster.latest_observation.confidence,
                            bbox_center_x_normalized=(
                                cluster.latest_observation.bbox_center_x
                                / cluster.latest_observation.image_width
                            ),
                            bbox_center_y_normalized=(
                                cluster.latest_observation.bbox_center_y
                                / cluster.latest_observation.image_height
                            ),
                            box_coverage=cluster.latest_observation.box_coverage,
                        )
                    )

            return tuple(candidates)

    def process_frame(self, frame: ObservationFrame) -> bool:
        """
        Incorporate one previously unprocessed detector frame.

        The source sequence and timestamp prevent the 20 Hz mission dispatcher
        from repeatedly counting the same camera frame as new evidence.
        """

        frame_key = (frame.sequence, frame.stamp_seconds)

        with self._lock:
            if frame_key == self._last_processed_frame:
                return False

            processing_time = frame.received_at

            if self._latest_processed_time is not None:
                processing_time = max(
                    processing_time,
                    self._latest_processed_time,
                )

            self._decay_all(processing_time)
            used_candidate_ids = set()

            for observation in frame.observations:
                if observation.class_id not in self._coverage_weights:
                    continue

                cluster = self._nearest_available_cluster(
                    observation,
                    used_candidate_ids,
                )

                if cluster is None:
                    cluster = self._create_cluster(
                        observation,
                        processing_time,
                    )
                else:
                    self._update_cluster_position(cluster, observation)
                    cluster.last_seen_seconds = processing_time
                    cluster.latest_observation = observation

                    if (
                        processing_time - cluster.last_contribution_seconds
                        >= self._minimum_observation_interval
                    ):
                        cluster.score += self._observation_contribution(
                            observation
                        )
                        cluster.last_contribution_seconds = processing_time

                cluster.observation_count += 1
                used_candidate_ids.add(cluster.candidate_id)

            self._last_processed_frame = frame_key
            self._latest_processed_time = processing_time
            return True

    def advance_time(self, current_time: float):
        """
        Decay evidence even when no new detector frame has arrived.

        This prevents evidence from remaining frozen if the camera or detector
        stops publishing.
        """

        if not math.isfinite(current_time):
            raise ValueError("Evidence time must be finite")

        with self._lock:
            if (
                self._latest_processed_time is not None
                and current_time < self._latest_processed_time
            ):
                return

            self._decay_all(current_time)
            self._latest_processed_time = current_time

    def _decay_all(self, current_time: float):
        """Decay every candidate according to elapsed real time."""

        for class_clusters in self._clusters.values():
            for cluster in class_clusters.values():
                elapsed = current_time - cluster.last_updated_seconds

                if elapsed <= 0:
                    continue

                cluster.score *= 0.5 ** (
                    elapsed / self._evidence_half_life
                )
                cluster.last_updated_seconds = current_time

    def _nearest_available_cluster(
        self,
        observation: RawTargetObservation,
        used_candidate_ids,
    ):
        """
        Find the closest unused candidate of the same class.

        Horizontal map distance is used because candidates represent locations
        on the search area. The measured Z position remains available for
        centring and later uncertainty handling.
        """

        closest_cluster = None
        closest_distance = None

        for cluster in self._clusters[observation.class_id].values():
            if cluster.candidate_id in used_candidate_ids:
                continue

            distance = math.hypot(
                observation.map_x - cluster.map_x,
                observation.map_y - cluster.map_y,
            )

            if distance > self._cluster_radius:
                continue

            if closest_distance is None or distance < closest_distance:
                closest_cluster = cluster
                closest_distance = distance

        return closest_cluster

    def _create_cluster(
        self,
        observation: RawTargetObservation,
        current_time: float,
    ) -> _EvidenceCluster:
        """Create a new class-specific spatial candidate."""

        candidate_id = self._next_candidate_id
        self._next_candidate_id += 1

        cluster = _EvidenceCluster(
            candidate_id=candidate_id,
            class_id=observation.class_id,
            map_x=observation.map_x,
            map_y=observation.map_y,
            map_z=observation.map_z,
            score=self._observation_contribution(observation),
            observation_count=0,
            last_seen_seconds=current_time,
            last_updated_seconds=current_time,
            last_contribution_seconds=current_time,
            latest_observation=observation,
        )

        self._clusters[observation.class_id][candidate_id] = cluster
        return cluster

    def _update_cluster_position(
        self,
        cluster: _EvidenceCluster,
        observation: RawTargetObservation,
    ):
        """Update the candidate position using a running observation average."""

        new_count = cluster.observation_count + 1
        previous_weight = cluster.observation_count / float(new_count)
        new_weight = 1.0 / float(new_count)

        cluster.map_x = (
            cluster.map_x * previous_weight
            + observation.map_x * new_weight
        )
        cluster.map_y = (
            cluster.map_y * previous_weight
            + observation.map_y * new_weight
        )
        cluster.map_z = (
            cluster.map_z * previous_weight
            + observation.map_z * new_weight
        )

    def _observation_contribution(
        self,
        observation: RawTargetObservation,
    ) -> float:
        """Calculate one bounded confidence-and-coverage contribution."""

        coverage_band = min(
            self.COVERAGE_BAND_COUNT - 1,
            int(observation.box_coverage * self.COVERAGE_BAND_COUNT),
        )
        coverage_weight = self._coverage_weights[
            observation.class_id
        ][coverage_band]

        return min(
            self._maximum_observation_contribution,
            observation.confidence * coverage_weight,
        )
