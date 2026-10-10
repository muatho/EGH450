"""
ROS communication boundary for the optional Ground Control veto.

This module records whether Ground Control is acknowledging newly received
camera images and exchanges candidate-specific deployment decisions. It does
not decide whether deployment is safe, required or permitted.
"""

import math
import threading
import time

from enum import Enum, auto
from typing import Optional

import rospy

from std_msgs.msg import Header
from spar_node.msg import DeploymentVetoRequest, DeploymentVetoResponse


class OperatorDecision(Enum):
    """Mission-facing result of the current optional operator request."""

    NONE = auto()
    DEPLOY = auto()
    VETO = auto()


class OperatorInterface:
    """
    Exchange image receipts and deployment decisions with Ground Control.

    A repeated acknowledgement containing the same source-image header does
    not refresh reception status. Ground Control must acknowledge newly
    received images to remain fresh.
    """

    def __init__(self):
        self._lock = threading.RLock()

        self._image_receipt_topic = rospy.get_param(
            "~gcs_image_receipt_topic",
            "/mission/gcs/image_receipt",
        )
        self._request_topic = rospy.get_param(
            "~deployment_veto_request_topic",
            "/mission/deployment_veto_request",
        )
        self._response_topic = rospy.get_param(
            "~deployment_veto_response_topic",
            "/mission/deployment_veto_response",
        )

        self._latest_image_identity = None
        self._latest_image_receipt_at = None

        self._active_candidate_id = None
        self._active_class_id = None
        self._decision = OperatorDecision.NONE

        self._request_publisher = rospy.Publisher(
            self._request_topic,
            DeploymentVetoRequest,
            queue_size=1,
        )
        self._image_receipt_subscriber = rospy.Subscriber(
            self._image_receipt_topic,
            Header,
            self._image_receipt_callback,
            queue_size=1,
        )
        self._response_subscriber = rospy.Subscriber(
            self._response_topic,
            DeploymentVetoResponse,
            self._response_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Operator interface using request %s, response %s and image receipt %s",
            self._request_topic,
            self._response_topic,
            self._image_receipt_topic,
        )

    def _image_receipt_callback(self, message: Header):
        """Record acknowledgement of one newly received GCS image."""

        image_identity = (
            int(message.seq),
            message.stamp.to_nsec(),
            message.frame_id,
        )

        with self._lock:
            if image_identity == self._latest_image_identity:
                return

            self._latest_image_identity = image_identity
            self._latest_image_receipt_at = time.monotonic()

    def _response_callback(self, message: DeploymentVetoResponse):
        """Accept a valid response matching the current candidate request."""

        with self._lock:
            if self._active_candidate_id is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Ignoring operator response because no veto request is active",
                )
                return

            if (
                message.candidate_id != self._active_candidate_id
                or message.class_id != self._active_class_id
            ):
                rospy.logwarn_throttle(
                    2.0,
                    "Ignoring operator response for a different candidate",
                )
                return

            if self._decision != OperatorDecision.NONE:
                return

            if message.decision == DeploymentVetoResponse.DEPLOY:
                self._decision = OperatorDecision.DEPLOY
                rospy.loginfo(
                    "Operator approved deployment for candidate %i",
                    message.candidate_id,
                )
                return

            if message.decision == DeploymentVetoResponse.VETO:
                self._decision = OperatorDecision.VETO
                rospy.logwarn(
                    "Operator vetoed deployment for candidate %i",
                    message.candidate_id,
                )
                return

            rospy.logwarn_throttle(
                2.0,
                "Ignoring invalid operator decision value: %i",
                message.decision,
            )

    def image_receipt_age(self) -> Optional[float]:
        """Return seconds since acknowledgement of a newly received GCS image."""

        with self._lock:
            if self._latest_image_receipt_at is None:
                return None

            return time.monotonic() - self._latest_image_receipt_at

    def has_fresh_imagery(self, maximum_age: float) -> bool:
        """Return whether Ground Control recently acknowledged a new image."""

        if not math.isfinite(maximum_age) or maximum_age <= 0:
            raise ValueError(
                "maximum_age must be finite and greater than zero"
            )

        receipt_age = self.image_receipt_age()
        return receipt_age is not None and receipt_age <= maximum_age

    def begin_veto_request(
        self,
        candidate_id: int,
        class_id: int,
        response_timeout: float,
    ):
        """Publish one candidate-specific optional deployment decision request."""

        if candidate_id <= 0:
            raise ValueError("candidate_id must be greater than zero")

        if class_id < 0:
            raise ValueError("class_id cannot be negative")

        if not math.isfinite(response_timeout) or response_timeout <= 0:
            raise ValueError(
                "response_timeout must be finite and greater than zero"
            )

        with self._lock:
            self._active_candidate_id = int(candidate_id)
            self._active_class_id = int(class_id)
            self._decision = OperatorDecision.NONE

        request = DeploymentVetoRequest()
        request.header.stamp = rospy.Time.now()
        request.candidate_id = candidate_id
        request.class_id = class_id
        request.response_timeout = rospy.Duration.from_sec(
            response_timeout
        )

        self._request_publisher.publish(request)

        rospy.loginfo(
            "Requested optional operator decision for candidate %i with %.1f-second timeout",
            candidate_id,
            response_timeout,
        )

    def decision(self) -> OperatorDecision:
        """Return the current candidate-specific operator decision."""

        with self._lock:
            return self._decision

    def clear_request(self):
        """Forget the completed or expired operator request."""

        with self._lock:
            self._active_candidate_id = None
            self._active_class_id = None
            self._decision = OperatorDecision.NONE