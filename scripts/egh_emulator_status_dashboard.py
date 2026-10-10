#!/usr/bin/env python3

import os
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


class Dashboard:
    def __init__(self):
        self.state_at = 0.0
        self.pose_at = 0.0
        self.odom_at = 0.0
        self.battery_at = 0.0
        self.battery = None

        rospy.Subscriber("/uavasr/state", State, self.state_callback, queue_size=1)
        rospy.Subscriber("/uavasr/pose", PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber("/uavasr/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/uavasr/battery", BatteryState, self.battery_callback, queue_size=1)

    def state_callback(self, _message):
        self.state_at = time.monotonic()

    def pose_callback(self, _message):
        self.pose_at = time.monotonic()

    def odom_callback(self, _message):
        self.odom_at = time.monotonic()

    def battery_callback(self, message):
        self.battery = message
        self.battery_at = time.monotonic()

    @staticmethod
    def registrations():
        try:
            publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
            nodes = {node for _, node_list in publishers + subscribers for node in node_list}
            topics = {topic for topic, _ in publishers}
            service_names = {service for service, _ in services}
            return nodes, topics, service_names
        except Exception:
            return set(), set(), set()

    def run(self):
        rate = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            now = time.monotonic()
            nodes, topics, services = self.registrations()

            master = "READY" if nodes or topics or services else "FAILED"
            breadcrumb = "READY" if "/breadcrumb/request_path" in services else "FAILED"
            emulator = "RUNNING" if "/uavasr" in nodes else "FAILED"
            state = "RECEIVING" if now - self.state_at < 5.0 else "NO DATA"
            spar = "READY" if "/spar/flight/status" in topics else "FAILED"
            pose = "RECEIVING" if now - self.pose_at < 5.0 else "NO DATA"
            odom = "RECEIVING" if now - self.odom_at < 5.0 else "NO DATA"
            depthai = "READY" if any("depthai" in node or "dai_publisher" in node for node in nodes) else "WAITING"

            if self.battery is None or now - self.battery_at >= 5.0:
                battery = "NO DATA"
            elif 0.0 <= self.battery.percentage <= 1.0:
                battery = "{:.0f}%".format(self.battery.percentage * 100.0)
            else:
                battery = "INVALID"

            os.write(1, b"\033[3J\033[2J\033[H")
            print("EGH450 EMULATOR  {}".format(time.strftime("%H:%M:%S")))
            print("Master: {:<10} Breadcrumb: {}".format(master, breadcrumb))
            print("Emulator: {:<8} State: {}".format(emulator, state))
            print("SPAR: {:<12} Pose: {}".format(spar, pose))
            print("Odom: {:<12} Battery: {}".format(odom, battery))
            print("DepthAI: {:<9} Mission: MANUAL START".format(depthai))

            if master != "READY":
                print("ACTION: Inspect the roscore pane.")
            elif emulator != "RUNNING" or state != "RECEIVING":
                print("ACTION: Inspect the emulator pane.")
            elif spar != "READY":
                print("ACTION: Inspect spar_uavasr.launch.")
            elif breadcrumb != "READY":
                print("ACTION: Inspect the Breadcrumb pane.")
            elif pose != "RECEIVING" or odom != "RECEIVING":
                print("ACTION: Check /uavasr/pose and /uavasr/odom.")
            elif battery in ("NO DATA", "INVALID"):
                print("WARN: Emulator battery telemetry {}.".format(battery.lower()))
            else:
                print("Supporting emulator systems appear ready.")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("egh_emulator_status_dashboard", anonymous=True)
    Dashboard().run()
