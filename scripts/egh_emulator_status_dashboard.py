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
        self.state_at = self.pose_at = self.odom_at = self.battery_at = 0.0
        self.battery = None
        rospy.Subscriber('/uavasr/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/uavasr/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/uavasr/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/uavasr/battery', BatteryState, self.battery_cb, queue_size=1)

    def state_cb(self, _): self.state_at = time.monotonic()
    def pose_cb(self, _): self.pose_at = time.monotonic()
    def odom_cb(self, _): self.odom_at = time.monotonic()
    def battery_cb(self, msg):
        self.battery, self.battery_at = msg, time.monotonic()

    @staticmethod
    def registration():
        try:
            pubs, subs, services = rosgraph.Master(rospy.get_name()).getSystemState()
            nodes = {n for _, ns in pubs + subs for n in ns}
            return nodes, {t for t, _ in pubs}, {s for s, _ in services}
        except Exception:
            return set(), set(), set()

    def run(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            now = time.monotonic()
            nodes, topics, services = self.registration()
            master = 'READY' if nodes or topics or services else 'FAILED'
            breadcrumb = 'READY' if '/breadcrumb/request_path' in services else 'FAILED'
            emulator = 'RUNNING' if '/uavasr' in nodes else 'FAILED'
            state = 'RECEIVING' if now - self.state_at < 5 else 'NO DATA'
            spar = 'READY' if '/spar/flight/status' in topics else 'FAILED'
            pose = 'RECEIVING' if now - self.pose_at < 5 else 'NO DATA'
            odom = 'RECEIVING' if now - self.odom_at < 5 else 'NO DATA'
            depthai = 'READY' if any('depthai' in n or 'dai_publisher' in n for n in nodes) else 'WAITING'
            if self.battery is None or now - self.battery_at >= 5:
                battery = 'NO DATA'
            elif 0 <= self.battery.percentage <= 1:
                battery = '{:.0f}%'.format(self.battery.percentage * 100)
            else:
                battery = 'INVALID'

            os.write(1, b'\033[3J\033[2J\033[H')
            print('EGH450 EMULATOR  {}'.format(time.strftime('%H:%M:%S')))
            print('Master: {:<10} Breadcrumb: {}'.format(master, breadcrumb))
            print('Emulator: {:<8} State: {}'.format(emulator, state))
            print('SPAR: {:<12} Pose: {}'.format(spar, pose))
            print('Odom: {:<12} Battery: {}'.format(odom, battery))
            print('DepthAI: {:<9} Mission: MANUAL START'.format(depthai))
            if master != 'READY':
                print('ACTION: Inspect the roscore pane.')
            elif emulator != 'RUNNING' or state != 'RECEIVING':
                print('ACTION: Inspect the emulator pane.')
            elif spar != 'READY':
                print('ACTION: Inspect spar_uavasr.launch.')
            elif breadcrumb != 'READY':
                print('ACTION: Inspect the Breadcrumb pane.')
            elif pose != 'RECEIVING' or odom != 'RECEIVING':
                print('ACTION: Check /uavasr/pose and /uavasr/odom.')
            elif battery in ('NO DATA', 'INVALID'):
                print('WARN: Emulator battery telemetry {}.'.format(battery.lower()))
            else:
                print('Supporting emulator systems appear ready.')
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('egh_emulator_status_dashboard', anonymous=True)
    Dashboard().run()
