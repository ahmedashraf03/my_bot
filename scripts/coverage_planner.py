#!/usr/bin/env python3
"""
Coverage Planner - Working version with room detection and simple zigzag
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ContactsState


class CoveragePlanner(Node):
    
    COVERAGE_WIDTH = 0.35  # meters
    MIN_FREE_CELLS = 200
    
    def __init__(self):
        super().__init__('coverage_planner')
        
        # QoS for map
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(ContactsState, '/bumper', self.bumper_callback, 10)
        
        # Publishers
        self._done_pub = self.create_publisher(Bool, '/coverage_done', 1)
        
        # Navigation
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.waypoints = []
        self.current_wp = 0
        self.path_started = False
        self.navigating = False
        
        self.get_logger().info('Coverage Planner started')

    def map_callback(self, msg):
        if self.path_started:
            return
            
        self.path_started = True
        
        # Get map data
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        resolution = msg.info.resolution
        origin = msg.info.origin
        
        # Find free space (value < 50)
        free_space = data < 50
        
        # Detect rooms using improved morphological operations
        rooms = self.detect_rooms(free_space)
        self.get_logger().info(f'Detected {len(rooms)} rooms')
        
        # Generate waypoints room by room
        all_waypoints = []
        for i, room_mask in enumerate(rooms):
            room_waypoints = self.generate_room_waypoints(room_mask, resolution, origin)
            if room_waypoints:
                all_waypoints.extend(room_waypoints)
                self.get_logger().info(f'Room {i+1}: {len(room_waypoints)} waypoints')
        
        self.waypoints = all_waypoints
        self.current_wp = 0
        
        self.get_logger().info(f'Total: {len(all_waypoints)} waypoints across all rooms')
        
        # Start navigation
        self.send_next_waypoint()
    
    def detect_rooms(self, free_space):
        """Detect separate rooms using moderate morphological operations"""
        from scipy import ndimage
        
        # Create smaller structuring element for moderate opening
        kernel = np.ones((3, 3), dtype=bool)
        
        # Perform moderate morphological opening to separate rooms
        # This will break narrow doorways but still allow passage
        opened = ndimage.binary_opening(free_space, structure=kernel, iterations=1)
        
        # No additional erosion - keep doorways partially open for navigation
        
        # Now use connected components on the processed space
        labeled, num_features = ndimage.label(opened)
        
        rooms = []
        for i in range(1, num_features + 1):  # Skip label 0 (background)
            room_mask = (labeled == i)
            # Filter out very small rooms (noise)
            if np.sum(room_mask) > 100:  # Minimum 100 cells
                rooms.append(room_mask)
        
        # Sort rooms by size (largest first) for better coverage
        rooms.sort(key=lambda x: np.sum(x), reverse=True)
        return rooms
    
    def generate_room_waypoints(self, room_mask, resolution, origin):
        """Generate zigzag waypoints for a single room"""
        # Get bounds of this room
        y_indices = np.where(np.any(room_mask, axis=1))[0]
        x_indices = np.where(np.any(room_mask, axis=0))[0]
        
        if len(y_indices) == 0 or len(x_indices) == 0:
            return []
            
        min_y = int(y_indices[0])
        max_y = int(y_indices[-1])
        
        # Generate zigzag waypoints for this room
        step_y = max(1, int(self.COVERAGE_WIDTH / resolution))
        waypoints = []
        direction = 1  # 1 = right, -1 = left
        
        for y in range(min_y, max_y + 1, step_y):
            # Find free cells in this row within the room
            row_free = np.where(room_mask[y, :])[0]
            if len(row_free) == 0:
                continue
                
            # Get leftmost and rightmost free cells in this room
            x_left = int(row_free[0])
            x_right = int(row_free[-1])
            
            # Convert to world coordinates
            wx_left = origin.position.x + x_left * resolution
            wx_right = origin.position.x + x_right * resolution
            wy = origin.position.y + y * resolution
            
            # Add waypoints in current direction
            if direction == 1:
                waypoints.append(self.create_pose(wx_left, wy))
                waypoints.append(self.create_pose(wx_right, wy))
            else:
                waypoints.append(self.create_pose(wx_right, wy))
                waypoints.append(self.create_pose(wx_left, wy))
                
            direction *= -1  # Switch direction for next row
        
        return waypoints
    
    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def send_next_waypoint(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('Coverage complete!')
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)
            return
            
        if not self._nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available!')
            return
            
        wp = self.waypoints[self.current_wp]
        goal = NavigateToPose.Goal()
        goal.pose = wp
        
        self.get_logger().info(f'Going to waypoint {self.current_wp}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')
        
        self.navigating = True
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Waypoint {self.current_wp} rejected, skipping to next')
            self.current_wp += 1
            self.navigating = False
            self.send_next_waypoint()
            return
            
        self.get_logger().info(f'Waypoint {self.current_wp} accepted, navigating...')
        handle.get_result_async().add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        self.navigating = False
        try:
            result = future.result()
            self.get_logger().info(f'Waypoint {self.current_wp} reached')
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')
            
        # Move to next waypoint
        self.current_wp += 1
        self.send_next_waypoint()
    
    def bumper_callback(self, msg):
        if len(msg.states) == 0 or not self.navigating:
            return
            
        self.get_logger().warn('Bumper hit! Skipping current waypoint')
        
        # Cancel current goal and skip to next
        self.current_wp += 1
        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()