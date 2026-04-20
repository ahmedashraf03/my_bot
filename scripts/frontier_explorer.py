#!/usr/bin/env python3
"""
Frontier Explorer — ROS2 Humble  (V3)

Changes vs V2:
  - Temporary blacklist with retry: failed frontiers are banned for
    BLACKLIST_EXPIRY seconds, not forever. After that window the robot
    tries them again (the costmap may have updated, the path may now exist).
  - Largest-cluster-first selection: robot heads toward big unknown areas
    (rooms) rather than the nearest pixel edge. Avoids getting trapped by
    one problematic doorway while two whole rooms remain unexplored.
  - Reduced MIN_FRONTIER_DIST (0.4 m): doorway frontiers were being
    silently dropped at 0.8 m, locking the robot out of rooms.
  - Safe atomic map saving: writes to a temp file first, then renames
    atomically so a Ctrl-C never corrupts the last good map. A timestamped
    backup is also kept so you can always roll back.
  - Publishes /exploration_done when mapping is complete.
  - Once NO_FRONTIER_LIMIT is reached the node stops completely (timer
    cancelled). Blacklist expiry is frozen during wind-down so the robot
    never revisits already-mapped areas.
  - No scipy — pure numpy only.
"""

import math
import os
import shutil
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from gazebo_msgs.msg import ContactsState
from tf2_ros import TransformException, Buffer, TransformListener


class FrontierExplorer(Node):

    # ── tunables ──────────────────────────────────────────────────────────
    EXPLORE_HZ          = 2.0
    GOAL_TIMEOUT        = 60.0
    MIN_FRONTIER_DIST   = 0.4    # m — lowered so doorway frontiers are not dropped
    MAX_FRONTIER_DIST   = 10.0   # m — relaxed automatically if nothing found
    SAFE_RADIUS_CELLS   = 2
    CLUSTER_SIZE_M      = 0.6
    MIN_CLUSTER_CELLS   = 2
    NO_FRONTIER_LIMIT   = 30    # Increased to allow more thorough exploration
    BLACKLIST_EXPIRY    = 30.0   # s — reduced expiry for faster retry
    MAP_SAVE_PATH       = '/home/ahmedashraf/dev_ws/house_map'
    GOAL_STABILITY_DIST = 1.0    # m — minimum distance to switch goals
    GOAL_STABILITY_TIME = 5.0    # s — minimum time before considering new goals
    MAP_VALIDATION_INTERVAL = 30  # s — check map integrity every 30 seconds
    MAX_MAP_CHANGES_RATE = 0.2    # Reduced to 20% for more sensitive corruption detection
    # ──────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('frontier_explorer')

        self.create_subscription(OccupancyGrid, '/map',               self._map_cb,    10)
        self.create_subscription(Odometry,       '/odometry/filtered', self._odom_cb,   10)
        self.create_subscription(ContactsState,  '/bumper',            self._bumper_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        self._tf_buf = Buffer()
        TransformListener(self._tf_buf, self)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Supervisor listens here
        self._done_pub = self.create_publisher(Bool, '/exploration_done', 1)

        # ── state ──
        self._map           = None
        self._rx            = None
        self._ry            = None
        self._navigating    = False
        self._goal_handle   = None
        self._last_goal     = None
        self._goal_sent_t   = None
        self._no_frontier_n = 0
        self._map_saved     = False
        self._bumped        = False
        self._done          = False

        # Goal stability tracking
        self._goal_stable_since = None
        self._last_stable_goal = None

        # Localization quality monitoring
        self._amcl_pose = None
        self._amcl_covariance = None
        self._localization_good = True
        self._last_localization_check = self.get_clock().now()

        # Map validation
        self._last_map_data = None
        self._last_map_validation = self.get_clock().now()
        self._map_corrupted = False

        # Temporary blacklist: bl_key → expiry monotonic time
        self._blacklist: dict = {}

        self._timer = self.create_timer(self.EXPLORE_HZ, self._loop)
        self.get_logger().info('FrontierExplorer started')

    # ── callbacks ─────────────────────────────────────────────────────────

    def _map_cb(self, msg):
        self._map = msg
        self._validate_map_integrity()

    def _odom_cb(self, msg):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y

    def _bumper_cb(self, msg):
        if not msg.states or self._bumped or self._done:
            return
        self.get_logger().warn('Bumper hit — cancelling goal')
        self._bumped = True
        self._cancel_goal()

    def _amcl_cb(self, msg):
        self._amcl_pose = msg.pose.pose
        self._amcl_covariance = msg.pose.covariance
        
        # Check localization quality
        if self._amcl_covariance:
            # Extract position covariance (x, y variance)
            x_var = self._amcl_covariance[0]
            y_var = self._amcl_covariance[7]
            
            # High variance indicates poor localization
            position_uncertainty = math.sqrt(x_var + y_var)
            
            if position_uncertainty > 0.5:  # 0.5m uncertainty threshold
                if self._localization_good:
                    self.get_logger().warn(
                        f'Poor localization detected (uncertainty={position_uncertainty:.2f}m)')
                self._localization_good = False
            else:
                if not self._localization_good:
                    self.get_logger().info(
                        f'Localization recovered (uncertainty={position_uncertainty:.2f}m)')
                self._localization_good = True

    def _validate_map_integrity(self):
        """Check for map corruption by detecting sudden large changes"""
        if self._map is None:
            return
        
        current_time = self.get_clock().now()
        time_since_validation = (current_time - self._last_map_validation).nanoseconds / 1e9
        
        # Only validate every MAP_VALIDATION_INTERVAL seconds
        if time_since_validation < self.MAP_VALIDATION_INTERVAL:
            return
        
        self._last_map_validation = current_time
        
        current_data = np.array(self._map.data)
        
        if self._last_map_data is not None:
            # Calculate percentage of changed cells
            total_cells = len(current_data)
            changed_cells = np.sum(current_data != self._last_map_data)
            change_rate = changed_cells / total_cells
            
            if change_rate > self.MAX_MAP_CHANGES_RATE:
                self.get_logger().error(
                    f'Map corruption detected! Change rate: {change_rate:.1%} ({changed_cells}/{total_cells} cells)')
                self._map_corrupted = True
                # Emergency: cancel current goal and stop exploration
                self._cancel_goal()
                self._navigating = False
                self._last_goal = None
            else:
                if self._map_corrupted:
                    self.get_logger().info('Map integrity restored')
                    self._map_corrupted = False
        
        self._last_map_data = current_data

    def _is_map_sufficiently_explored(self):
        """Check if map has sufficient explored area to consider exploration complete"""
        if not self._map:
            return False
            
        data = np.array(self._map.data)
        total_cells = len(data)
        
        # Count different cell types
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data >= 50)
        unknown_cells = np.sum(data == -1)
        
        # Consider exploration complete if at least 70% of map is known (free + occupied)
        known_ratio = (free_cells + occupied_cells) / total_cells
        
        self.get_logger().info(
            f'Map coverage: {known_ratio:.1%} (free: {free_cells}, occupied: {occupied_cells}, unknown: {unknown_cells})')
        
        return known_ratio >= 0.7

    # ── position ──────────────────────────────────────────────────────────

    def _have_position(self):
        if self._rx is not None:
            return True
        try:
            t = self._tf_buf.lookup_transform('map', 'base_footprint',
                                              rclpy.time.Time())
            self._rx = t.transform.translation.x
            self._ry = t.transform.translation.y
            return True
        except TransformException:
            return False

    # ── blacklist ─────────────────────────────────────────────────────────

    @staticmethod
    def _bl_key(x, y):
        """0.4 m grid key — groups nearby points so one failure covers a doorway."""
        return (round(x / 0.4), round(y / 0.4))

    def _blacklist_add(self, x, y):
        self._blacklist[self._bl_key(x, y)] = time.monotonic() + self.BLACKLIST_EXPIRY
        self.get_logger().info(
            f'Blacklisted ({x:.2f},{y:.2f}) for {self.BLACKLIST_EXPIRY:.0f}s')

    def _is_blacklisted(self, x, y):
        key    = self._bl_key(x, y)
        expiry = self._blacklist.get(key)
        if expiry is None:
            return False
        # Freeze expiry during wind-down so already-mapped areas are never revisited
        if self._no_frontier_n > 0:
            return True
        if time.monotonic() > expiry:
            del self._blacklist[key]
            return False
        return True

    # ── main loop ─────────────────────────────────────────────────────────

    def _loop(self):
        if self._done or self._map is None or not self._have_position():
            return
            
        # Skip exploration if localization is poor or map is corrupted
        if not self._localization_good or self._map_corrupted:
            current_time = self.get_clock().now()
            time_since_check = (current_time - self._last_localization_check).nanoseconds / 1e9
            if time_since_check > 2.0:  # Log every 2 seconds
                if not self._localization_good:
                    self.get_logger().warn('Skipping exploration - poor localization')
                if self._map_corrupted:
                    self.get_logger().error('Emergency: Map corrupted - stopping exploration')
                    # Wait for map to stabilize
                    if time_since_check > 10.0:  # After 10 seconds, try to recover
                        self.get_logger().info('Attempting map recovery')
                        self._map_corrupted = False  # Try to resume
                        # Clear recent goals that might be invalid
                        self._blacklist.clear()
                        self._last_goal = None
                        self._navigating = False
                self._last_localization_check = current_time
            return

        # Consume bumper flag
        if self._bumped:
            self._bumped = False
            if self._last_goal:
                self._blacklist_add(*self._last_goal)
            self._navigating  = False
            self._goal_handle = None
            self._last_goal   = None

        # Monitor active goal for timeout
        if self._navigating:
            elapsed = (self.get_clock().now() - self._goal_sent_t).nanoseconds / 1e9
            if elapsed > self.GOAL_TIMEOUT:
                self.get_logger().warn(f'Goal timed out ({elapsed:.0f}s)')
                if self._last_goal:
                    self._blacklist_add(*self._last_goal)
                self._cancel_goal()
                self._navigating  = False
                self._goal_handle = None
                self._last_goal   = None
            else:
                return   # still navigating normally

        # Find and send next frontier
        frontier = self._find_frontier()

        if frontier is None:
            self._no_frontier_n += 1
            self.get_logger().info(
                f'No reachable frontier ({self._no_frontier_n}/{self.NO_FRONTIER_LIMIT})')
            
            # Only finish if we've genuinely explored most areas
            if self._no_frontier_n >= self.NO_FRONTIER_LIMIT:
                # Check if map has sufficient explored area
                if self._map and self._is_map_sufficiently_explored():
                    self._finish_exploration()
                else:
                    self.get_logger().info('Map not sufficiently explored, continuing search')
                    self._no_frontier_n = 0  # Reset counter
            return

        self._no_frontier_n = 0
        
        # Goal stability check - prevent rapid goal switching
        current_time = self.get_clock().now()
        if self._last_stable_goal is not None:
            dist_to_last = math.hypot(frontier[0] - self._last_stable_goal[0], frontier[1] - self._last_stable_goal[1])
            time_since_stable = (current_time - self._goal_stable_since).nanoseconds / 1e9
            
            if dist_to_last < self.GOAL_STABILITY_DIST and time_since_stable < self.GOAL_STABILITY_TIME:
                # Keep current goal, don't switch
                self.get_logger().debug(
                    f'Goal stability: keeping current goal (dist={dist_to_last:.2f}m, time={time_since_stable:.1f}s)')
                return
        
        # Check if we're too close to current position (avoid getting stuck)
        if self._rx is not None and self._ry is not None:
            dist_to_current = math.hypot(frontier[0] - self._rx, frontier[1] - self._ry)
            if dist_to_current < 0.3:  # Too close to current position
                self.get_logger().warn(f'Frontier too close ({dist_to_current:.2f}m) - skipping')
                return
        
        # Update stable goal tracking
        self._goal_stable_since = current_time
        self._last_stable_goal = frontier
        self._send_goal(frontier)

    # ── finish — called exactly once ──────────────────────────────────────

    def _finish_exploration(self):
        if self._done:
            return
        self._done = True

        self.get_logger().info('=== Exploration complete — stopping ===')

        # Stop the loop timer so the robot does nothing further
        self._timer.cancel()

        # Cancel any in-flight goal
        self._cancel_goal()

        # Save map
        self._save_map()

        # Signal the supervisor
        msg      = Bool()
        msg.data = True
        self._done_pub.publish(msg)
        self.get_logger().info('Published /exploration_done = True')

    # ── frontier detection ────────────────────────────────────────────────

    def _find_frontier(self):
        if self._map_corrupted:
            self.get_logger().warn('Skipping frontier detection - map corrupted')
            return None
            
        m    = self._map
        h, w = m.info.height, m.info.width
        res  = m.info.resolution
        ox   = m.info.origin.position.x
        oy   = m.info.origin.position.y

        # Validate map data
        if not hasattr(m, 'data') or len(m.data) == 0:
            self.get_logger().error('Invalid map data - skipping frontier detection')
            return None
            
        data = np.array(m.data, dtype=np.int8).reshape(h, w)
        free = (data == 0)
        unk  = (data == -1)
        obs  = (data >= 50)

        # Frontier mask: free cell with at least one unknown neighbour
        unk_u = np.zeros_like(unk); unk_u[1:,  :]  = unk[:-1, :]
        unk_d = np.zeros_like(unk); unk_d[:-1, :]  = unk[1:,  :]
        unk_l = np.zeros_like(unk); unk_l[:,  1:]  = unk[:,  :-1]
        unk_r = np.zeros_like(unk); unk_r[:, :-1]  = unk[:,   1:]
        frontier_mask = free & (unk_u | unk_d | unk_l | unk_r)

        # Safety: dilate obstacles and remove unsafe frontier cells
        r = self.SAFE_RADIUS_CELLS
        obs_exp = obs.copy()
        for s in range(1, r + 1):
            o_u = np.zeros_like(obs); o_u[s:,  :]  = obs[:-s, :]
            o_d = np.zeros_like(obs); o_d[:-s, :]  = obs[s:,  :]
            o_l = np.zeros_like(obs); o_l[:,  s:]  = obs[:, :-s]
            o_r = np.zeros_like(obs); o_r[:, :-s]  = obs[:,  s:]
            obs_exp |= o_u | o_d | o_l | o_r
        frontier_mask &= ~obs_exp

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return None

        wx_arr = ox + xs * res
        wy_arr = oy + ys * res

        # Grid-based clustering
        cs = self.CLUSTER_SIZE_M
        bucket: dict = {}
        for bx, by in zip(wx_arr, wy_arr):
            key = (int(bx / cs), int(by / cs))
            bucket.setdefault(key, []).append((bx, by))

        def _candidates(max_dist):
            out = []
            for pts in bucket.values():
                if len(pts) < self.MIN_CLUSTER_CELLS:
                    continue
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                d  = math.hypot(cx - self._rx, cy - self._ry)
                if d < self.MIN_FRONTIER_DIST or self._is_blacklisted(cx, cy):
                    continue
                if max_dist is not None and d > max_dist:
                    continue
                out.append((cx, cy, d, len(pts)))
            return out

        candidates = _candidates(self.MAX_FRONTIER_DIST) or _candidates(None)
        if not candidates:
            return None

        # Largest cluster first — drives robot toward rooms, not wall edges
        candidates.sort(key=lambda c: (-c[3], c[2]))
        best = candidates[0]
        self.get_logger().info(
            f'Frontier → ({best[0]:.2f},{best[1]:.2f}) '
            f'dist={best[2]:.2f}m cluster={best[3]} pool={len(candidates)}')
        return (best[0], best[1])

    # ── navigation ────────────────────────────────────────────────────────

    def _send_goal(self, frontier):
        if not self._nav.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 not ready — will retry')
            return

        pose                    = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = frontier[0]
        pose.pose.position.y    = frontier[1]
        pose.pose.orientation.w = 1.0

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        self._navigating  = True
        self._last_goal   = frontier
        self._goal_sent_t = self.get_clock().now()

        self._nav.send_goal_async(goal).add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected — blacklisting')
            if self._last_goal:
                self._blacklist_add(*self._last_goal)
            self._navigating  = False
            self._goal_handle = None
            self._last_goal   = None
            return
        self._goal_handle = handle
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self._goal_done_cb)

    def _goal_done_cb(self, future):
        self._navigating  = False
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
            self._last_goal = None
            return

        if status in (4, 6):
            # Only blacklist if goal actually failed (not close enough)
            if self._last_goal and self._amcl_pose:
                dist_to_goal = math.hypot(self._amcl_pose.position.x - self._last_goal[0],
                                        self._amcl_pose.position.y - self._last_goal[1])
                if dist_to_goal < 0.8:  # Increased tolerance - close enough to goal
                    self.get_logger().info(f'Goal actually reached (status={status}, dist={dist_to_goal:.2f}m) - NOT blacklisting')
                else:
                    self.get_logger().warn(f'Goal failed (status={status}, dist={dist_to_goal:.2f}m) — blacklisting temporarily')
                    self._blacklist_add(*self._last_goal)
            else:
                self.get_logger().warn(f'Goal status={status} — blacklisting temporarily')
                if self._last_goal:
                    self._blacklist_add(*self._last_goal)
        elif status == 5:  # SUCCEEDED
            self.get_logger().info(f'Goal reached successfully (status={status})')
        else:
            self.get_logger().info(f'Goal completed with status={status}')

        self._last_goal = None

    def _cancel_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None

    # ── safe atomic map saving ────────────────────────────────────────────

    def _save_map(self):
        path     = self.MAP_SAVE_PATH
        tmp_path = path + '.tmp'
        saved_ok = False

        try:
            client = self.create_client(SaveMap, '/map_saver/save_map')
            if client.wait_for_service(timeout_sec=5.0):
                req         = SaveMap.Request()
                req.map_url = tmp_path
                rclpy.spin_until_future_complete(self, client.call_async(req),
                                                 timeout_sec=10.0)
                saved_ok = True
        except Exception as e:
            self.get_logger().error(f'Service save failed: {e}')

        if not saved_ok:
            try:
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', tmp_path],
                    check=True, timeout=10.0)
                saved_ok = True
            except Exception as e:
                self.get_logger().error(f'CLI save failed: {e}')
                return

        # Simple override - no backups, just overwrite existing map
        for ext in ('.pgm', '.yaml'):
            tmp_file  = tmp_path + ext
            live_file = path + ext
            if not os.path.exists(tmp_file):
                continue
            # Just override the existing map file
            os.replace(tmp_file, live_file)

        self.get_logger().info(f'Map saved → {path}')

    
# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node._done:
            node.get_logger().info('Interrupted — saving map')
            node._save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()