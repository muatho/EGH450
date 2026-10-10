"""ROS communication boundary for the separately launched payload node."""

import threading
import time

import rospy

from std_msgs.msg import Bool


class PayloadInterface:
    """Command each class-specific payload once and observe actuation replies."""

    PERSON_CLASS_ID = 0
    BACKPACK_CLASS_ID = 1

    def __init__(self):
        self._lock = threading.RLock()
        self._enabled = bool(rospy.get_param("~deploy", True))

        self._actuator_by_class = {
            self.PERSON_CLASS_ID: "a",
            self.BACKPACK_CLASS_ID: "b",
        }
        self._command_topics = {
            "a": rospy.get_param(
                "~payload_a_command_topic", "/actuator_control/actuator_a"
            ),
            "b": rospy.get_param(
                "~payload_b_command_topic", "/actuator_control/actuator_b"
            ),
        }
        self._status_topics = {
            "a": rospy.get_param(
                "~payload_a_status_topic", "/actuator_control/actuator_a/deployed"
            ),
            "b": rospy.get_param(
                "~payload_b_status_topic", "/actuator_control/actuator_b/deployed"
            ),
        }

        self._deployed = {"a": False, "b": False}
        self._requested = {"a": False, "b": False}

        self._publishers = {
            actuator: rospy.Publisher(topic, Bool, queue_size=1)
            for actuator, topic in self._command_topics.items()
        }
        self._status_subscribers = [
            rospy.Subscriber(
                self._status_topics["a"],
                Bool,
                self._status_callback,
                callback_args="a",
                queue_size=1,
            ),
            rospy.Subscriber(
                self._status_topics["b"],
                Bool,
                self._status_callback,
                callback_args="b",
                queue_size=1,
            ),
        ]

        if self._enabled:
            rospy.loginfo("Payload deployment interface enabled")
        else:
            rospy.logwarn("Payload deployment is disabled")

    @property
    def enabled(self) -> bool:
        """Return whether this mission permits physical payload commands."""
        return self._enabled

    def _status_callback(self, message: Bool, actuator: str):
        """Record the separately running payload node's physical result."""
        with self._lock:
            self._deployed[actuator] = bool(message.data)

        if message.data:
            rospy.loginfo("Payload %s reported deployed", actuator.upper())

    def wait_until_ready(self, timeout_seconds: float) -> bool:
        """Wait for both payload command subscribers when deployment is enabled."""
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be greater than zero")

        if not self._enabled:
            return True

        deadline = time.monotonic() + timeout_seconds
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if all(
                publisher.get_num_connections() > 0
                for publisher in self._publishers.values()
            ):
                rospy.loginfo("Payload controller subscribers are available")
                return True

            rate.sleep()

        rospy.logerr("Payload controller unavailable after %.1f seconds", timeout_seconds)
        return False

    def class_is_available(self, class_id: int) -> bool:
        """Return whether an undeployed payload exists for this target class."""
        actuator = self._actuator_by_class.get(class_id)

        if actuator is None:
            return False

        with self._lock:
            return not self._deployed[actuator] and not self._requested[actuator]

    def request_deployment(self, class_id: int) -> bool:
        """Send one non-blocking open command for the target's assigned payload."""
        actuator = self._actuator_by_class.get(class_id)

        if not self.class_is_available(class_id) or actuator is None:
            return False

        with self._lock:
            self._requested[actuator] = True

            if not self._enabled:
                self._deployed[actuator] = True

        if not self._enabled:
            rospy.logwarn(
                "Simulating payload %s deployment for target class %i; no physical command sent",
                actuator.upper(),
                class_id,
            )
            return True

        rospy.logwarn(
            "Commanding payload %s for target class %i",
            actuator.upper(),
            class_id,
        )
        self._publishers[actuator].publish(Bool(data=True))
        return True

    def class_is_deployed(self, class_id: int) -> bool:
        """Return whether the assigned payload node acknowledged deployment."""
        actuator = self._actuator_by_class.get(class_id)

        if actuator is None:
            return False

        with self._lock:
            return self._deployed[actuator]

    def all_payloads_deployed(self) -> bool:
        """Return whether both class-specific payloads were acknowledged."""
        with self._lock:
            return all(self._deployed.values())
