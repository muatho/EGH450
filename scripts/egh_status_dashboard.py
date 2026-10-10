#!/usr/bin/env python3
import os
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState


class Dashboard:
    def __init__(self):
        self.state = None
        self.state_at = 0.0
        self.pose_at = 0.0
        self.battery = None
        self.battery_at = 0.0

        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped,
            self.on_pose, queue_size=1)
        rospy.Subscriber(
            "/mavros/battery", BatteryState,
            self.on_battery, queue_size=1)

    def on_state(self, message):
        self.state = message
        self.state_at = time.monotonic()

    def on_pose(self, _message):
        self.pose_at = time.monotonic()

    def on_battery(self, message):
        self.battery = message
        self.battery_at = time.monotonic()

    @staticmethod
    def registration():
        try:
            publishers, subscribers, services = rosgraph.Master(
                rospy.get_name()).getSystemState()
            nodes = {node for _, providers in publishers for node in providers}
            nodes.update(
                node for _, providers in subscribers for node in providers)
            topics = {topic for topic, _ in publishers}
            service_names = {service for service, _ in services}
            return nodes, topics, service_names
        except Exception:
            return set(), set(), set()

    def run(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            now = time.monotonic()
            nodes, topics, services = self.registration()

            master = "READY" if nodes or topics or services else "FAILED"
            breadcrumb = (
                "READY" if "/breadcrumb/request_path" in services else "FAILED")
            mavros = "RUNNING" if "/mavros" in nodes else "FAILED"
            pixhawk = (
                "CONNECTED"
                if self.state is not None
                and self.state.connected
                and now - self.state_at < 5.0
                else "DISCONNECTED")
            spar = (
                "READY" if "/spar/flight/status" in topics else "FAILED")
            pose = (
                "RECEIVING" if now - self.pose_at < 5.0 else "NO DATA")
            depthai = (
                "READY"
                if any("depthai" in node or "dai_publisher" in node
                       for node in nodes)
                else "WAITING")

            if self.battery is None or now - self.battery_at >= 5.0:
                battery = "NO DATA"
            elif 0.0 <= self.battery.percentage <= 1.0:
                battery = "{:.0f}%".format(self.battery.percentage * 100.0)
            else:
                battery = "INVALID"

            os.write(1, b"\033[3J\033[2J\033[H")
            print("EGH450 STATUS  {}".format(time.strftime("%H:%M:%S")))
            print("Master: {:<10} Breadcrumb: {}".format(master, breadcrumb))
            print("MAVROS: {:<10} Pixhawk: {}".format(mavros, pixhawk))
            print("SPAR:   {:<10} Pose: {}".format(spar, pose))
            print("Battery: {:<9} DepthAI: {}".format(battery, depthai))
            print("Mission: MANUAL START")

            if master != "READY":
                print("ACTION: Inspect the roscore pane.")
            elif pixhawk != "CONNECTED":
                print("ACTION: Check power/USB/control pane.")
            elif spar != "READY":
                print("ACTION: Inspect control.launch.")
            elif breadcrumb != "READY":
                print("ACTION: Inspect the Breadcrumb pane.")
            elif pose != "RECEIVING":
                print("ACTION: Check Vicon/environment.")
            elif battery in ("NO DATA", "INVALID"):
                print("WARN: Battery telemetry {}.".format(battery.lower()))
            else:
                print("Supporting systems appear ready.")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("egh_status_dashboard", anonymous=True)
    Dashboard().run()
