#!/usr/bin/env python3
"""Vocalise map-frame YOLO and ArUco targets at the GCS."""
import queue
import shutil
import subprocess
import threading

import rospy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray

from depthai_publisher.object_announcer_core import TargetAnnouncementGate, announcement_text


class SpeechWorker:
    """Serialize speech requests without blocking ROS callbacks."""

    def __init__(self, command, rate=8, volume=0, pitch=15,
                 pitch_range=20, language="en", voice_type="female1"):
        self.command = shutil.which(command)
        if self.command is None:
            raise RuntimeError("Speech command not found: {}".format(command))
        self.rate = int(rate)
        self.volume = int(volume)
        self.pitch = int(pitch)
        self.pitch_range = int(pitch_range)
        self.language = str(language)
        self.voice_type = str(voice_type)
        self.messages = queue.Queue()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name="target-speech", daemon=True)
        self.thread.start()

    def say(self, text):
        self.messages.put_nowait(text)

    def _run(self):
        while not self.stopping.is_set():
            try:
                text = self.messages.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                subprocess.run([
                    self.command, "--wait", "--rate", str(self.rate),
                    "--volume", str(self.volume),
                    "--pitch", str(self.pitch),
                    "--pitch-range", str(self.pitch_range),
                    "--language", self.language,
                    "--voice-type", self.voice_type,
                    text,
                ], check=True)
            except (OSError, subprocess.CalledProcessError) as error:
                rospy.logerr_throttle(5.0, "Target announcement failed: %s", error)
            finally:
                self.messages.task_done()

    def shutdown(self):
        self.stopping.set()
        self.thread.join(timeout=1.0)


class ObjectAnnouncer:
    """Report confirmed global targets audibly and on a GCS text topic."""

    def __init__(self):
        self.object_topic = rospy.get_param("~object_topic", "/objects/observations")
        self.aruco_topic = rospy.get_param("~aruco_topic", "/world_model/aruco_markers")
        self.labels = [str(item) for item in rospy.get_param("~labels", ["person", "backpack"])]
        self.minimum_confidence = float(rospy.get_param("~minimum_confidence", 0.8))
        radius = float(rospy.get_param("~match_radius", 1.0))
        cooldown = float(rospy.get_param("~announcement_cooldown", 60.0))
        self.object_gate = TargetAnnouncementGate(
            rospy.get_param("~object_confirmation_frames", 3), radius,
            cooldown_seconds=cooldown,
        )
        self.aruco_gate = TargetAnnouncementGate(
            rospy.get_param("~aruco_confirmation_frames", 2), radius,
            deduplicate_by_type=True,
            cooldown_seconds=cooldown,
        )
        self.aruco_stamps = {}
        self.speech = SpeechWorker(
            rospy.get_param("~speech_command", "spd-say"),
            rospy.get_param("~speech_rate", 8),
            rospy.get_param("~speech_volume", 0),
            rospy.get_param("~speech_pitch", 15),
            rospy.get_param("~speech_pitch_range", 20),
            rospy.get_param("~speech_language", "en"),
            rospy.get_param("~speech_voice_type", "female1"),
        )
        self.alert_publisher = rospy.Publisher(
            "/gcs/target_alert", String, queue_size=10, latch=True
        )
        self.object_subscriber = rospy.Subscriber(
            self.object_topic, Detection3DArray, self.object_callback, queue_size=1
        )
        self.aruco_subscriber = rospy.Subscriber(
            self.aruco_topic, MarkerArray, self.aruco_callback, queue_size=1
        )
        rospy.on_shutdown(self.speech.shutdown)
        rospy.loginfo("GCS target announcer listening on %s and %s", self.object_topic, self.aruco_topic)

    def report(self, target_type, position):
        phrase = announcement_text(target_type, position)
        self.announce(phrase)

    def announce(self, phrase):
        rospy.loginfo(phrase)
        self.alert_publisher.publish(String(data=phrase))
        self.speech.say(phrase)

    def object_callback(self, message):
        if message.header.frame_id not in ("map", "/map"):
            rospy.logwarn_throttle(5.0, "Ignoring object frame '%s'; expected map", message.header.frame_id)
            return
        observations = []
        for detection in message.detections:
            if not detection.results:
                continue
            result = max(detection.results, key=lambda item: item.score)
            class_id = int(result.id)
            if result.score < self.minimum_confidence:
                continue
            if class_id < 0 or class_id >= len(self.labels):
                rospy.logwarn_throttle(5.0, "Ignoring unknown target class ID %d", class_id)
                continue
            point = result.pose.pose.position
            observations.append((self.labels[class_id], (point.x, point.y, point.z)))
        for target_type, position in self.object_gate.update(observations):
            self.report(target_type, position)

    def aruco_callback(self, message):
        fresh = []
        for marker in message.markers:
            if marker.header.frame_id not in ("map", "/map"):
                continue
            stamp = marker.header.stamp.to_nsec()
            if self.aruco_stamps.get(marker.id) == stamp:
                continue
            self.aruco_stamps[marker.id] = stamp
            point = marker.pose.position
            fresh.append(("ArUco marker {}".format(marker.id), (point.x, point.y, point.z)))
        for target_type, position in self.aruco_gate.update(fresh):
            self.report(target_type, position)


def main():
    rospy.init_node("object_announcer")
    ObjectAnnouncer()
    rospy.spin()


if __name__ == "__main__":
    main()
