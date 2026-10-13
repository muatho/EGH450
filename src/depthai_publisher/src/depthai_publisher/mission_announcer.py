#!/usr/bin/env python3
"""Independent GCS voice alerts for arming and mission landing events."""
import queue
import shutil
import subprocess
import threading

import rospy
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool, String

from depthai_publisher.mission_announcer_core import (
    mission_event_announcement,
    payload_announcement,
)


class SpeechWorker:
    def __init__(self, command="spd-say", rate=8, volume=0, pitch=15,
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
        self.thread = threading.Thread(target=self._run, daemon=True)
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
                rospy.logerr_throttle(5.0, "Mission announcement failed: %s", error)
            finally:
                self.messages.task_done()

    def shutdown(self):
        self.stopping.set()
        self.thread.join(timeout=1.0)


class MissionAnnouncer:
    def __init__(self):
        self.last_armed = None
        self.events = set()
        self.speech = SpeechWorker(
            rospy.get_param("~speech_command", "spd-say"),
            rospy.get_param("~speech_rate", 8),
            rospy.get_param("~speech_volume", 0),
            rospy.get_param("~speech_pitch", 15),
            rospy.get_param("~speech_pitch_range", 20),
            rospy.get_param("~speech_language", "en"),
            rospy.get_param("~speech_voice_type", "female1"),
        )
        self.publisher = rospy.Publisher(
            "/gcs/mission_alert", String, queue_size=10, latch=True
        )
        self.state_subscriber = rospy.Subscriber(
            "/mavros/state", State, self.state_callback, queue_size=1
        )
        self.log_subscriber = rospy.Subscriber(
            "/rosout_agg", Log, self.log_callback, queue_size=20
        )
        self.payload_a_subscriber = rospy.Subscriber(
            "/actuator_control/actuator_a", Bool,
            self.payload_a_callback, queue_size=1
        )
        self.payload_b_subscriber = rospy.Subscriber(
            "/actuator_control/actuator_b", Bool,
            self.payload_b_callback, queue_size=1
        )
        rospy.on_shutdown(self.speech.shutdown)
        rospy.loginfo("Independent GCS mission announcer ready")

    def announce(self, phrase):
        rospy.loginfo(phrase)
        self.publisher.publish(String(data=phrase))
        self.speech.say(phrase)

    def state_callback(self, message):
        armed = bool(message.armed)
        if self.last_armed is None:
            self.last_armed = armed
            if armed:
                self.events.clear()
                self.announce("Aircraft armed.")
            return
        if armed == self.last_armed:
            return
        if armed:
            self.events.clear()
            self.announce("Aircraft armed.")
        else:
            self.announce("Aircraft disarmed.")
        self.last_armed = armed

    def log_callback(self, message):
        event = mission_event_announcement(message.name, message.msg)
        if event is None:
            return
        key, phrase = event

        # Mission start is the reset boundary even if the aircraft was already
        # armed and therefore produced no new arming-state transition.
        if key == "mission_start":
            self.events.clear()

        if key in self.events:
            return
        self.events.add(key)
        self.announce(phrase)

    def payload_a_callback(self, message):
        self.payload_callback(message, "a")

    def payload_b_callback(self, message):
        self.payload_callback(message, "b")

    def payload_callback(self, message, actuator):
        """Announce each payload deployment command received."""

        # False closes or resets the servo; only True represents deployment.
        if not bool(message.data):
            return

        event = payload_announcement(actuator)

        if event is None:
            return

        # Each True message is a new deployment command. The mission controller
        # already prevents the same payload being deployed twice in one mission.
        _, phrase = event
        self.announce(phrase)


def main():
    rospy.init_node("mission_voice_announcer")
    MissionAnnouncer()
    rospy.spin()


if __name__ == "__main__":
    main()
