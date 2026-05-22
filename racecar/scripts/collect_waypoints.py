#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waypoint collection tool.
Press 's' to save current AMCL pose to waypoints.csv.
Press 'q' to quit.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys, select, termios, tty, math, os

CSV_PATH = "/home/racecar/src/waypoints.csv"


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WaypointCollector(Node):
    def __init__(self):
        super().__init__("collect_waypoints")
        self.pose = None
        self.settings = termios.tcgetattr(sys.stdin)

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10,
        )
        self.get_logger().info("Waypoint collector started")
        self.get_logger().info("Press 's' to save pose, 'q' to quit")

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def save_waypoint(self):
        if self.pose is None:
            self.get_logger().warn("No pose data received yet")
            return

        x = self.pose.position.x
        y = self.pose.position.y
        q = self.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw)

        is_new = not os.path.exists(CSV_PATH)
        with open(CSV_PATH, "a") as f:
            if is_new:
                f.write("# x, y, yaw_rad, yaw_deg\n")
            f.write(f"{x:.3f}, {y:.3f}, {yaw:.3f}, {yaw_deg:.1f}\n")

        self.get_logger().info(f"Saved waypoint ({x:.2f}, {y:.2f}, {yaw_deg:.1f} deg)")

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                key = self.get_key()

                if key == "s":
                    self.save_waypoint()
                elif key == "q":
                    self.get_logger().info("Done, exiting")
                    break
                elif key == "\x03":
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = WaypointCollector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
