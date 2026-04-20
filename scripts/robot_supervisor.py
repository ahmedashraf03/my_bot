#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import subprocess
import time
import math
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class RobotSupervisor(Node):

    def __init__(self):

        super().__init__('robot_supervisor')

        # Detect if running on real robot
        self.is_simulation = os.getenv('IS_SIMULATION', 'true').lower() == 'true'
        self.get_logger().info(f'Running on {"simulation" if self.is_simulation else "real robot"}')

        self.mode = "cleaning"
        self.map_received = False

        self.slam_process = None
        self.frontier_process = None
        self.coverage_process = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Real robot specific settings
        self.exploration_timeout = 300.0 if not self.is_simulation else 180.0  # 5 min for real, 3 min for sim
        self.frontier_retry_limit = 3 if not self.is_simulation else 5  # More retries for real robot

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.exploration_done_sub = self.create_subscription(
            Bool,
            '/exploration_done',
            self.exploration_done_callback,
            10)

        self.get_logger().info(f'Robot supervisor started ({"simulation" if self.is_simulation else "real robot"}')

        # Auto-detect if we need to start exploration or cleaning
        self.auto_start_delay = 5.0  # Wait before starting operations
        self.create_timer(self.auto_start_delay, self._auto_start_operations)

    # ------------------------------------------------

    def pose_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def start_cleaning(self):

        self.get_logger().info("Starting cleaning mode")

        self.coverage_process = subprocess.Popen([
            "ros2",
            "run",
            "my_bot",
            "coverage_planner.py"
        ])

    # ------------------------------------------------

    def map_callback(self, msg):

        if not self.map_received:
            self.get_logger().info("Map received")
            self.map_received = True

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        for y in range(1, height-1):
            for x in range(1, width-1):

                index = y * width + x

                # free cell
                if data[index] != 0:
                    continue

                neighbors = [
                    (x+1,y),(x-1,y),(x,y+1),(x,y-1)
                ]

                for nx,ny in neighbors:

                    n_index = ny * width + nx

                    # frontier = free cell next to unknown
                    if data[n_index] == -1 and self.mode == "cleaning":

                        self.get_logger().info("Frontier detected → switching to exploration")

                        self.start_exploration()
                        return

    # ------------------------------------------------

    def exploration_done_callback(self, msg):
        if msg.data and self.mode == "exploration":
            self.get_logger().info("Received exploration completion signal")
            self.finish_exploration()

    # ------------------------------------------------
    def start_exploration(self):
        self.get_logger().info("Exploration mode triggered")

        self.mode = "exploration"

        self.get_logger().info("Stopping cleaning")

        if self.coverage_process:
            self.coverage_process.terminate()

        subprocess.call(["pkill", "-f", "amcl"])

        time.sleep(2)

        self.get_logger().info("Starting SLAM")

        self.slam_process = subprocess.Popen([
            "ros2",
            "launch",
            "my_bot",
            "online_async_launch.py",
            "params_file:=/home/ahmedashraf/dev_ws/src/my_bot/config/mapper_params_online_async.yaml",
            "use_sim_time:=true"
        ])

        time.sleep(5)

        self.get_logger().info("Starting frontier exploration")

        self.frontier_process = subprocess.Popen([
            "ros2",
            "run",
            "my_bot",
            "frontier_explorer.py"
        ])

        self.get_logger().info("Waiting for exploration to complete...")
        # Exploration will be terminated by the /exploration_done callback

    # ------------------------------------------------

    def finish_exploration(self):

        self.get_logger().info("Stopping exploration")

        if self.frontier_process:
            self.frontier_process.terminate()

        if self.slam_process:
            self.slam_process.terminate()

        time.sleep(2)

        self.get_logger().info("Saving map")

        subprocess.call([
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            "/home/ahmedashraf/dev_ws/house_map"
        ])

        time.sleep(2)

        self.get_logger().info("Restarting localization")

        subprocess.Popen([
            "ros2",
            "launch",
            "my_bot",
            "localization_launch.py",
            "map:=/home/ahmedashraf/dev_ws/house_map.yaml",
            "use_sim_time:=true"
        ])

        time.sleep(5)

        self.start_cleaning()

        self.mode = "cleaning"

# ------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = RobotSupervisor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()