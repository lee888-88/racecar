#!/usr/bin/env python3
import rclpy, math, os, time
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

CSV_PATH = "/home/racecar/src/waypoints.csv"


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def load_waypoints():
    if not os.path.exists(CSV_PATH):
        print("Error: " + CSV_PATH + " not found")
        return []

    waypoints = []
    with open(CSV_PATH, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                continue
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2])

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            q = yaw_to_quat(yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            waypoints.append(pose)

    return waypoints


STATUS_NAMES = {
    1: "UNKNOWN",
    2: "SUCCEEDED",
    3: "ABORTED",
    4: "CANCELED",
}


class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def send_and_wait(self, pose):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info("Sending goal...")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return False

        self.get_logger().info("Goal accepted, navigating...")

        result_future = goal_handle.get_result_async()
        self._done = False
        self._status = None

        def cb(future):
            self._status = future.result().status
            self._done = True

        result_future.add_done_callback(cb)

        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.5)

        name = STATUS_NAMES.get(self._status, f"UNKNOWN({self._status})")
        self.get_logger().info("Result: " + name)

        return self._status == 2  # SUCCEEDED


def main():
    waypoints = load_waypoints()
    if not waypoints:
        print("Error: no waypoints found")
        return

    print(f"Loaded {len(waypoints)} waypoints")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i+1}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})")

    rclpy.init()
    node = NavNode()

    for i, wp in enumerate(waypoints):
        print(f"\n--- Waypoint {i+1}/{len(waypoints)} ---")
        ok = node.send_and_wait(wp)
        if not ok:
            print(f"Failed at waypoint {i+1}, stopping")
            break
        print(f"Reached waypoint {i+1}")

    print("\nAll waypoints completed")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# import rclpy, math, os, time
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from nav2_msgs.action import NavigateThroughPoses
# from geometry_msgs.msg import PoseStamped
# from action_msgs.msg import GoalStatus

# CSV_PATH = "/home/racecar/src/waypoints.csv"


# def yaw_to_quat(yaw):
#     half = yaw * 0.5
#     return (0.0, 0.0, math.sin(half), math.cos(half))


# def load_waypoints():
#     if not os.path.exists(CSV_PATH):
#         print("Error: " + CSV_PATH + " not found")
#         return []

#     waypoints = []
#     with open(CSV_PATH, "r") as f:
#         for line in f:
#             line = line.strip()
#             if not line or line.startswith("#"):
#                 continue
#             parts = [p.strip() for p in line.split(",")]
#             if len(parts) < 3:
#                 continue
#             x = float(parts[0])
#             y = float(parts[1])
#             yaw = float(parts[2])

#             pose = PoseStamped()
#             pose.header.frame_id = "map"
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             q = yaw_to_quat(yaw)
#             pose.pose.orientation.x = q[0]
#             pose.pose.orientation.y = q[1]
#             pose.pose.orientation.z = q[2]
#             pose.pose.orientation.w = q[3]
#             waypoints.append(pose)

#     return waypoints


# class MultiNavNode(Node):
#     def __init__(self):
#         super().__init__("multi_nav")
#         self._action_client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
#         self._goal_handle = None

#     def send_goal(self, waypoints):
#         goal_msg = NavigateThroughPoses.Goal()
#         goal_msg.poses = waypoints

#         self._action_client.wait_for_server()
#         self._goal_handle = self._action_client.send_goal_async(goal_msg)
#         rclpy.spin_until_future_complete(self, self._goal_handle)
#         self._goal_handle = self._goal_handle.result()

#         if not self._goal_handle.accepted:
#             print("Goal rejected!")
#             return False

#         print("Goal accepted, navigating...")
#         return True

#     def wait_for_result(self):
#         if not self._goal_handle:
#             return None

#         result_future = self._goal_handle.get_result_async()
#         while rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.5)
#             if result_future.done():
#                 break
#             if self._goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
#                 break

#         rclpy.spin_until_future_complete(self, result_future)
#         status = result_future.result().status
#         return status


# def main():
#     waypoints = load_waypoints()
#     if not waypoints:
#         print("Error: no waypoints found. Run collect_waypoints.py first.")
#         return

#     print(f"Loaded {len(waypoints)} waypoints, starting navigation...")
#     for i, wp in enumerate(waypoints):
#         print(f"  Waypoint {i+1}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})")

#     rclpy.init()
#     node = MultiNavNode()

#     ok = node.send_goal(waypoints)
#     if ok:
#         status = node.wait_for_result()
#         status_names = {
#             GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
#             GoalStatus.STATUS_ABORTED: "ABORTED",
#             GoalStatus.STATUS_CANCELED: "CANCELED",
#         }
#         name = status_names.get(status, f"UNKNOWN({status})")
#         print(f"\nAll waypoints completed! Result: {name}")
#     else:
#         print("Navigation failed!")

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
