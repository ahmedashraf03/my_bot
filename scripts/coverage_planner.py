#!/usr/bin/env python3
"""
Coverage Planner - Early waypoint completion via proximity check
with integrated ROS2 hardware signaling for suction/working state.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

class CoveragePlanner(Node):
    COVERAGE_WIDTH = 0.35
    WALL_CLEARANCE = 0.50
    ARRIVAL_RADIUS = 0.35

    def __init__(self):
        super().__init__('coverage_planner')

        # ── ROS2 Publisher for Hardware Signals ────────────────────────
        self._suction_pub = self.create_publisher(Bool, '/suction_cmd', 10)

        map_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._done_pub = self.create_publisher(Bool, '/coverage_done', 1)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.waypoints = []
        self.current_wp = 0
        self.path_started = False
        self.navigating = False
        self._goal_handle = None

        self._proximity_timer = self.create_timer(0.1, self.proximity_check)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav.wait_for_server()
        self.get_logger().info('Nav2 ready. Coverage Planner started.')

    # ══════════════════════════════════════════════════════════════════════
    # ARDUINO SIGNAL CONTROL VIA ROS2 TOPIC
    # ══════════════════════════════════════════════════════════════════════
    def _signal_high(self):
        """Coverage started → Tell hardware interface to turn suction HIGH"""
        self.get_logger().info('Path execution started: Sending Suction ON signal.')
        self._suction_pub.publish(Bool(data=True))

    def _signal_low(self):
        """Coverage finished/stopped → Tell hardware interface to turn suction LOW"""
        self.get_logger().info('Path execution halted/finished: Sending Suction OFF signal.')
        self._suction_pub.publish(Bool(data=False))

    # ══════════════════════════════════════════════════════════════════════
    # ODOMETRY
    # ══════════════════════════════════════════════════════════════════════
    def odom_callback(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    # ══════════════════════════════════════════════════════════════════════
    # PROXIMITY CHECK
    # ══════════════════════════════════════════════════════════════════════
    def proximity_check(self):
        if not self.navigating or self.current_wp >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp]
        dx = self._robot_x - wp.pose.position.x
        dy = self._robot_y - wp.pose.position.y
        dist = (dx * dx + dy * dy) ** 0.5

        if dist < self.ARRIVAL_RADIUS:
            self.get_logger().info(
                f'Waypoint {self.current_wp} close enough ({dist:.2f} m), skipping'
            )
            self._advance_to_next()

    # ══════════════════════════════════════════════════════════════════════
    # MAP CALLBACK
    # ══════════════════════════════════════════════════════════════════════
    def map_callback(self, msg):
        if self.path_started:
            return

        self.path_started = True

        # ══ SET SIGNAL HIGH when coverage starts ══
        self._signal_high()

        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        resolution = msg.info.resolution
        origin = msg.info.origin

        free_space = data < 50
        rooms = self.detect_rooms(free_space)

        self.get_logger().info(f'Detected {len(rooms)} rooms')

        all_waypoints = []
        for i, room_mask in enumerate(rooms):
            wps = self.generate_room_waypoints(room_mask, resolution, origin)
            if wps:
                all_waypoints.extend(wps)
                self.get_logger().info(f'Room {i+1}: {len(wps)} waypoints')

        self.waypoints = all_waypoints
        self.current_wp = 0

        self.get_logger().info(f'Total: {len(all_waypoints)} waypoints across all rooms')
        self.send_next_waypoint()

    # ══════════════════════════════════════════════════════════════════════
    # ROOM DETECTION
    # ══════════════════════════════════════════════════════════════════════
    def detect_rooms(self, free_space):
        from scipy import ndimage
        kernel = np.ones((3, 3), dtype=bool)
        opened = ndimage.binary_opening(free_space, structure=kernel, iterations=1)
        labeled, num_features = ndimage.label(opened)

        rooms = []
        for i in range(1, num_features + 1):
            room_mask = (labeled == i)
            if np.sum(room_mask) > 100:
                rooms.append(room_mask)

        rooms.sort(key=lambda x: np.sum(x), reverse=True)
        return rooms

    # ══════════════════════════════════════════════════════════════════════
    # WAYPOINT GENERATION
    # ══════════════════════════════════════════════════════════════════════
    def generate_room_waypoints(self, room_mask, resolution, origin):
        y_indices = np.where(np.any(room_mask, axis=1))[0]
        if len(y_indices) == 0:
            return []

        min_y = int(y_indices[0])
        max_y = int(y_indices[-1])
        step_y = max(1, int(self.COVERAGE_WIDTH / resolution))
        clearance_cells = int(np.ceil(self.WALL_CLEARANCE / resolution))

        waypoints = []
        direction = 1

        for row_y in range(min_y, max_y + 1, step_y):
            band_lo = max(min_y, row_y - step_y // 2)
            band_hi = min(max_y, row_y + step_y // 2)
            band = room_mask[band_lo:band_hi + 1, :]
            col_any = np.any(band, axis=0)
            free_cols = np.where(col_any)[0]

            if len(free_cols) == 0:
                continue

            x_left = int(free_cols[0]) + clearance_cells
            x_right = int(free_cols[-1]) - clearance_cells

            if x_right <= x_left:
                continue

            row_y_clamped = np.clip(row_y, min_y + clearance_cells, max_y - clearance_cells)
            wy = origin.position.y + row_y_clamped * resolution
            wx_left = origin.position.x + x_left * resolution
            wx_right = origin.position.x + x_right * resolution

            if direction == 1:
                waypoints.append(self.create_pose(wx_left, wy))
                waypoints.append(self.create_pose(wx_right, wy))
            else:
                waypoints.append(self.create_pose(wx_right, wy))
                waypoints.append(self.create_pose(wx_left, wy))

            direction *= -1

        return waypoints

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    # ══════════════════════════════════════════════════════════════════════
    # NAVIGATION
    # ══════════════════════════════════════════════════════════════════════
    def send_next_waypoint(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('Coverage complete!')

            # ══ SET SIGNAL LOW when coverage finishes ══
            self._signal_low()

            self._done_pub.publish(Bool(data=True))
            return

        wp = self.waypoints[self.current_wp]
        goal = NavigateToPose.Goal()
        goal.pose = wp
        goal.behavior_tree = ''

        self.get_logger().info(
            f'Waypoint {self.current_wp}/{len(self.waypoints)}: '
            f'({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})'
        )

        self.navigating = True
        self._goal_handle = None
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Waypoint {self.current_wp} rejected, skipping')
            self.current_wp += 1
            self.navigating = False
            self.send_next_waypoint()
            return

        self._goal_handle = handle
        self.get_logger().info(f'Waypoint {self.current_wp} accepted')
        handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        if not self.navigating:
            return

        self.navigating = False
        self._goal_handle = None

        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Waypoint {self.current_wp} reached (Nav2 confirmed)')
            elif status == GoalStatus.STATUS_CANCELED:
                return
            else:
                self.get_logger().warn(f'Waypoint {self.current_wp} status {status}, skipping')
        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')

        self.current_wp += 1
        self.send_next_waypoint()

    def _advance_to_next(self):
        if not self.navigating:
            return

        self.navigating = False
        self.current_wp += 1

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # ══ SET SIGNAL LOW on shutdown safely via topic ══
        try:
            node._signal_low()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()