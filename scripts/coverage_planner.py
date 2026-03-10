#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CoveragePlanner(Node):

    def __init__(self):
        super().__init__('coverage_planner')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )

        self.nav_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        self.map_received = False

        self.get_logger().info("Coverage planner started")

    def map_callback(self, msg):

        if self.map_received:
            return

        self.map_received = True

        self.get_logger().info("Map received, generating optimized coverage path")

        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin = msg.info.origin
        data = msg.data

        step = int(0.35 / resolution)   # ~35cm cleaning width

        poses = []
        direction = 1

        for y in range(0, height, step):

            start_x = None
            end_x = None

            for x in range(width):

                index = y * width + x

                if data[index] == 0:

                    if start_x is None:
                        start_x = x

                    end_x = x

            if start_x is None:
                continue

            wx_start = start_x * resolution + origin.position.x
            wx_end = end_x * resolution + origin.position.x
            wy = y * resolution + origin.position.y

            pose1 = PoseStamped()
            pose1.header.frame_id = "map"
            pose1.pose.position.x = float(wx_start)
            pose1.pose.position.y = float(wy)
            pose1.pose.orientation.w = 1.0

            pose2 = PoseStamped()
            pose2.header.frame_id = "map"
            pose2.pose.position.x = float(wx_end)
            pose2.pose.position.y = float(wy)
            pose2.pose.orientation.w = 1.0

            if direction == 1:
                poses.append(pose1)
                poses.append(pose2)
            else:
                poses.append(pose2)
                poses.append(pose1)

            direction *= -1

        self.get_logger().info(f"Generated {len(poses)} optimized waypoints")

        self.send_waypoints(poses)

    def send_waypoints(self, poses):

        self.nav_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info("Sending coverage path to Nav2")

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Coverage path rejected")
            return

        self.get_logger().info("Coverage started")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        self.get_logger().info("Coverage completed")


def main(args=None):

    rclpy.init(args=args)

    node = CoveragePlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()