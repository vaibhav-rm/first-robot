#!/usr/bin/env python3
"""Waypoint navigator using Nav2 BasicNavigator.

Two modes (same node, parameter-driven):
  1. Static waypoints (NAVIGATION task): goes through WAYPOINTS in order,
     halting 3 seconds at each (as required), then waits for dynamic goals.
  2. Dynamic goals (NAVIGATION WITH ARUCO task): subscribes /Nav2_coordinates
     (geometry_msgs/Point published by aruco_detector) and navigates there,
     also halting 3 seconds on success.

Default static waypoints are approach poses in front of the 3 ArUco boards
so the forward camera can see each marker:
  (2.0, 1.2) -> in front of marker 1 (2.5, 1.5)
  (-2.0, -0.8) -> in front of marker 2 (-2.5, -1.0)
  (1.2, -2.2) -> in front of marker 3 (1.5, -2.8)
Override with ROS param `waypoints` (flat list [x1,y1,x2,y2,...]).
Set `run_static_mission:=false` to skip static waypoints and only serve
/Nav2_coordinates (useful for pure ArUco-chained runs).
"""
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


DEFAULT_WAYPOINTS = [(2.0, 1.2), (-2.0, -0.8), (1.2, -2.2)]


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.declare_parameter('run_static_mission', True)
        self.declare_parameter('waypoints', [c for wp in DEFAULT_WAYPOINTS for c in wp])
        self.declare_parameter('halt_secs', 3.0)

        self.callback_group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator(node_name='basic_navigator')
        self._nav_lock = threading.Lock()

        self.coord_sub = self.create_subscription(
            Point,
            '/Nav2_coordinates',
            self.dynamic_goal_callback,
            10,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Waypoint Navigator initialized (static + /Nav2_coordinates)')

    def set_initial_pose(self, x, y, yaw=0.0):
        import math
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(x)
        initial_pose.pose.position.y = float(y)
        initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.navigator.setInitialPose(initial_pose)

    def go_to_pose(self, x, y):
        """Blocking single-goal navigation with 3s halt on success."""
        with self._nav_lock:
            halt = float(self.get_parameter('halt_secs').value)
            self.get_logger().info(f'Navigating to: ({x}, {y})')
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(x)
            goal_pose.pose.position.y = float(y)
            goal_pose.pose.orientation.w = 1.0

            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                time.sleep(0.2)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Goal ({x}, {y}) reached! Halting {halt}s...')
                time.sleep(halt)
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')

    def run_static_mission(self):
        flat = list(self.get_parameter('waypoints').value)
        waypoints = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]
        if not waypoints:
            self.get_logger().warn('Empty waypoints param, skipping static mission')
            return
        self.get_logger().info(f'Starting static mission: {len(waypoints)} waypoints')
        for x, y in waypoints:
            self.go_to_pose(x, y)
        self.get_logger().info('Static mission done. Waiting for /Nav2_coordinates goals...')

    def dynamic_goal_callback(self, msg):
        self.get_logger().info(f'Received dynamic goal: ({msg.x}, {msg.y})')
        nav_thread = threading.Thread(target=self.go_to_pose, args=(msg.x, msg.y), daemon=True)
        nav_thread.start()


def main(args=None):
    rclpy.init(args=args)
    navigator_node = WaypointNavigator()

    # Wait for Nav2 to be fully active before sending goals
    navigator_node.navigator.waitUntilNav2Active()

    run_static = bool(navigator_node.get_parameter('run_static_mission').value)
    if run_static:
        initial_thread = threading.Thread(target=navigator_node.run_static_mission, daemon=True)
        initial_thread.start()

    executor = MultiThreadedExecutor()
    executor.add_node(navigator_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
