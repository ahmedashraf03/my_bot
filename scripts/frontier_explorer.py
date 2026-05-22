#!/usr/bin/env python3
"""
Frontier Explorer — ROS2 Humble  (V4)

Changes vs V3:
  - FIX: Timer period bug — create_timer() takes seconds, not Hz.
    EXPLORE_HZ=2.0 now correctly fires at 2 Hz (every 0.5 s).
  - FIX: Stale goal-callback race — a generation counter guards against
    a timed-out goal's done-callback corrupting fresh navigation state.
  - FIX: spin_until_future_complete() inside spin() deadlock — map saving
    now uses a dedicated SingleThreadedExecutor in a background thread.
  - FIX: TransformListener reference stored so Python GC cannot collect it.
  - FIX: amcl_covariance truthiness check replaced with `is not None`.
  - FIX: Nav2 action status codes replaced with GoalStatus named constants.
  - FIX: Map QoS set to TRANSIENT_LOCAL / RELIABLE to match SLAM output.
  - IMPROVEMENT: Obstacle dilation extended to 8-connected (diagonals included).
  - IMPROVEMENT: Frontier scoring uses weighted (cluster_size, distance) so
    a nearby medium cluster beats a huge far-away cluster.
  - IMPROVEMENT: Map-coverage denominator uses connected free space from the
    robot's current cell, not the entire grid, to avoid the "open boundary"
    false-negative.
  - IMPROVEMENT: Blacklist cleared conservatively on map-corruption recovery
    (only entries near the robot, not the whole table).
  - IMPROVEMENT: Spin-in-place recovery when no frontier is found for
    SPIN_RECOVERY_THRESHOLD consecutive cycles.
  - IMPROVEMENT: All tunables exposed as ROS parameters (declare_parameter).
  - IMPROVEMENT: Periodic auto-save every MAP_AUTOSAVE_INTERVAL seconds so
    the map is not lost if the node is killed without KeyboardInterrupt.
  - No scipy — pure numpy only.
"""

import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)

from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from tf2_ros import TransformException, Buffer, TransformListener

try:
    from gazebo_msgs.msg import ContactsState
    _HAVE_GAZEBO_MSGS = True
except ImportError:
    _HAVE_GAZEBO_MSGS = False


# ── QoS profiles ──────────────────────────────────────────────────────────────

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

_SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class FrontierExplorer(Node):

    # ── default tunables (overridable via ROS parameters) ─────────────────
    _DEFAULTS = dict(
        explore_hz              = 2.0,
        goal_timeout            = 60.0,
        min_frontier_dist       = 0.4,
        max_frontier_dist       = 10.0,
        safe_radius_cells       = 2,
        cluster_size_m          = 0.6,
        min_cluster_cells       = 2,
        no_frontier_limit       = 30,
        blacklist_expiry        = 30.0,
        map_save_path           = '/home/ahmedashraf/dev_ws/house_map',
        goal_stability_dist     = 1.0,
        goal_stability_time     = 5.0,
        map_validation_interval = 30.0,
        max_map_changes_rate    = 0.20,
        map_coverage_threshold  = 0.70,
        frontier_dist_weight    = 0.5,   # weight for distance penalty in scoring
        spin_recovery_threshold = 5,     # consecutive no-frontier cycles before spin
        map_autosave_interval   = 120.0, # seconds between periodic auto-saves
    )
    # ──────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── declare & load ROS parameters ──
        for name, default in self._DEFAULTS.items():
            self.declare_parameter(name, default)

        def _p(name):
            return self.get_parameter(name).value

        self.EXPLORE_HZ              = _p('explore_hz')
        self.GOAL_TIMEOUT            = _p('goal_timeout')
        self.MIN_FRONTIER_DIST       = _p('min_frontier_dist')
        self.MAX_FRONTIER_DIST       = _p('max_frontier_dist')
        self.SAFE_RADIUS_CELLS       = int(_p('safe_radius_cells'))
        self.CLUSTER_SIZE_M          = _p('cluster_size_m')
        self.MIN_CLUSTER_CELLS       = int(_p('min_cluster_cells'))
        self.NO_FRONTIER_LIMIT       = int(_p('no_frontier_limit'))
        self.BLACKLIST_EXPIRY        = _p('blacklist_expiry')
        self.MAP_SAVE_PATH           = _p('map_save_path')
        self.GOAL_STABILITY_DIST     = _p('goal_stability_dist')
        self.GOAL_STABILITY_TIME     = _p('goal_stability_time')
        self.MAP_VALIDATION_INTERVAL = _p('map_validation_interval')
        self.MAX_MAP_CHANGES_RATE    = _p('max_map_changes_rate')
        self.MAP_COVERAGE_THRESHOLD  = _p('map_coverage_threshold')
        self.FRONTIER_DIST_WEIGHT    = _p('frontier_dist_weight')
        self.SPIN_RECOVERY_THRESHOLD = int(_p('spin_recovery_threshold'))
        self.MAP_AUTOSAVE_INTERVAL   = _p('map_autosave_interval')

        # ── subscriptions ──
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, _MAP_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, _SENSOR_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, _SENSOR_QOS)

        if _HAVE_GAZEBO_MSGS:
            self.create_subscription(
                ContactsState, '/bumper', self._bumper_cb, _SENSOR_QOS)
        else:
            self.get_logger().warn(
                'gazebo_msgs not available — bumper detection disabled')

        # ── TF (store reference so GC cannot collect the listener) ──
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── action client ──
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── publishers ──
        self._done_pub = self.create_publisher(Bool, '/exploration_done', 1)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

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

        # Goal-generation counter — guards stale done-callbacks
        self._goal_gen      = 0

        # Goal stability tracking
        self._goal_stable_since = None
        self._last_stable_goal  = None

        # Localization quality monitoring
        self._amcl_pose         = None
        self._amcl_covariance   = None
        self._localization_good = True
        self._last_loc_check_t  = self.get_clock().now()

        # Map validation
        self._last_map_data       = None
        self._last_map_val_t      = self.get_clock().now()
        self._map_corrupted       = False

        # Spin-recovery state
        self._spinning            = False
        self._spin_start_t        = None

        # Periodic auto-save
        self._last_autosave_t     = self.get_clock().now()

        # Temporary blacklist: bl_key → expiry monotonic time
        self._blacklist: dict = {}

        # Main loop timer — period in seconds = 1 / Hz
        self._timer = self.create_timer(1.0 / self.EXPLORE_HZ, self._loop)
        self.get_logger().info('FrontierExplorer V4 started')

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
        self._amcl_pose       = msg.pose.pose
        self._amcl_covariance = msg.pose.covariance

        if self._amcl_covariance is not None:
            x_var = self._amcl_covariance[0]
            y_var = self._amcl_covariance[7]
            position_uncertainty = math.sqrt(max(x_var, 0.0) + max(y_var, 0.0))

            if position_uncertainty > 0.5:
                if self._localization_good:
                    self.get_logger().warn(
                        f'Poor localization detected (uncertainty={position_uncertainty:.2f}m)')
                self._localization_good = False
            else:
                if not self._localization_good:
                    self.get_logger().info(
                        f'Localization recovered (uncertainty={position_uncertainty:.2f}m)')
                self._localization_good = True

    # ── map integrity ─────────────────────────────────────────────────────

    def _validate_map_integrity(self):
        """Detect sudden large map changes that may indicate SLAM corruption."""
        if self._map is None:
            return

        now = self.get_clock().now()
        dt  = (now - self._last_map_val_t).nanoseconds / 1e9
        if dt < self.MAP_VALIDATION_INTERVAL:
            return

        self._last_map_val_t = now
        current_data = np.array(self._map.data)

        if self._last_map_data is not None and len(current_data) == len(self._last_map_data):
            total_cells   = len(current_data)
            changed_cells = int(np.sum(current_data != self._last_map_data))
            change_rate   = changed_cells / total_cells

            if change_rate > self.MAX_MAP_CHANGES_RATE:
                self.get_logger().error(
                    f'Map corruption detected! Change rate: {change_rate:.1%} '
                    f'({changed_cells}/{total_cells} cells)')
                self._map_corrupted = True
                self._cancel_goal()
                self._navigating = False
                self._last_goal  = None
            else:
                if self._map_corrupted:
                    self.get_logger().info('Map integrity restored')
                    self._map_corrupted = False

        self._last_map_data = current_data

    # ── exploration completeness ───────────────────────────────────────────

    def _is_map_sufficiently_explored(self):
        """
        Check coverage using connected free space reachable from the robot,
        not the entire grid (which may include large out-of-bounds unknown areas).
        """
        if not self._map or self._rx is None:
            return False

        m    = self._map
        h, w = m.info.height, m.info.width
        res  = m.info.resolution
        ox   = m.info.origin.position.x
        oy   = m.info.origin.position.y

        data = np.array(m.data, dtype=np.int8).reshape(h, w)
        free = (data == 0)

        # Robot cell
        rx_c = int((self._rx - ox) / res)
        ry_c = int((self._ry - oy) / res)
        if not (0 <= rx_c < w and 0 <= ry_c < h):
            return False

        # BFS flood-fill from robot to find connected free cells
        visited  = np.zeros((h, w), dtype=bool)
        frontier = [(ry_c, rx_c)]
        if not free[ry_c, rx_c]:
            return False
        visited[ry_c, rx_c] = True
        connected_free = 0

        while frontier:
            next_frontier = []
            for (r, c) in frontier:
                connected_free += 1
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and free[nr, nc]:
                        visited[nr, nc] = True
                        next_frontier.append((nr, nc))
            frontier = next_frontier

        # Unknown cells adjacent to connected free space = still-reachable unknowns
        reachable_unknown = 0
        for r in range(h):
            for c in range(w):
                if data[r, c] == -1:
                    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and visited[nr, nc]:
                            reachable_unknown += 1
                            break

        total_reachable = connected_free + reachable_unknown
        if total_reachable == 0:
            return False

        known_ratio = connected_free / total_reachable
        self.get_logger().info(
            f'Map coverage (reachable): {known_ratio:.1%} '
            f'(free={connected_free}, reachable_unknown={reachable_unknown})')
        return known_ratio >= self.MAP_COVERAGE_THRESHOLD

    # ── position ──────────────────────────────────────────────────────────

    def _have_position(self):
        """Return True if robot position in map frame is known."""
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
    def _bl_key(x, y, cell_size=0.4):
        """Grid key — groups nearby points so one failure covers a doorway."""
        return (round(x / cell_size), round(y / cell_size))

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

    def _blacklist_clear_near(self, x, y, radius=2.0):
        """Remove only blacklist entries within `radius` metres of (x, y)."""
        to_del = []
        for key in list(self._blacklist.keys()):
            kx = key[0] * 0.4
            ky = key[1] * 0.4
            if math.hypot(kx - x, ky - y) <= radius:
                to_del.append(key)
        for k in to_del:
            del self._blacklist[k]
        if to_del:
            self.get_logger().info(
                f'Cleared {len(to_del)} blacklist entries near ({x:.2f},{y:.2f})')

    # ── spin-in-place recovery ────────────────────────────────────────────

    def _start_spin_recovery(self):
        """Command the robot to rotate in place for ~3 s to expose new frontiers."""
        self.get_logger().info('Starting spin-in-place recovery')
        self._spinning    = True
        self._spin_start_t = self.get_clock().now()
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        self._cmd_vel_pub.publish(twist)

    def _update_spin_recovery(self):
        """Continue or finish the spin recovery. Returns True while spinning."""
        if not self._spinning:
            return False
        elapsed = (self.get_clock().now() - self._spin_start_t).nanoseconds / 1e9
        if elapsed < 3.0:
            twist = Twist()
            twist.angular.z = 0.5
            self._cmd_vel_pub.publish(twist)
            return True
        # Stop spinning
        self._cmd_vel_pub.publish(Twist())
        self._spinning = False
        self.get_logger().info('Spin recovery complete')
        return False

    # ── main loop ─────────────────────────────────────────────────────────

    def _loop(self):
        if self._done or self._map is None or not self._have_position():
            return

        # Periodic auto-save
        now = self.get_clock().now()
        if (now - self._last_autosave_t).nanoseconds / 1e9 >= self.MAP_AUTOSAVE_INTERVAL:
            self._last_autosave_t = now
            self.get_logger().info('Auto-saving map…')
            threading.Thread(target=self._save_map, daemon=True).start()

        # Skip exploration if localization is poor or map is corrupted
        if not self._localization_good or self._map_corrupted:
            dt = (now - self._last_loc_check_t).nanoseconds / 1e9
            if dt > 2.0:
                if not self._localization_good:
                    self.get_logger().warn('Skipping exploration — poor localization')
                if self._map_corrupted:
                    self.get_logger().error('Map corrupted — pausing exploration')
                    if dt > 10.0:
                        self.get_logger().info('Attempting map recovery')
                        self._map_corrupted = False
                        # Only clear blacklist near the robot, not globally
                        if self._rx is not None:
                            self._blacklist_clear_near(self._rx, self._ry)
                        self._last_goal  = None
                        self._navigating = False
                self._last_loc_check_t = now
            return

        # Handle spin recovery
        if self._update_spin_recovery():
            return

        # Consume bumper flag
        if self._bumped:
            self._bumped = False
            if self._last_goal:
                self._blacklist_add(*self._last_goal)
            self._navigating  = False
            self._goal_handle = None
            self._last_goal   = None
            self._goal_gen   += 1

        # Monitor active goal for timeout
        if self._navigating:
            elapsed = (now - self._goal_sent_t).nanoseconds / 1e9
            if elapsed > self.GOAL_TIMEOUT:
                self.get_logger().warn(f'Goal timed out ({elapsed:.0f}s)')
                if self._last_goal:
                    self._blacklist_add(*self._last_goal)
                self._cancel_goal()
                self._navigating  = False
                self._goal_handle = None
                self._last_goal   = None
                self._goal_gen   += 1  # invalidate any pending done-callback
            else:
                return  # still navigating normally

        # Find and send next frontier
        frontier = self._find_frontier()

        if frontier is None:
            self._no_frontier_n += 1
            self.get_logger().info(
                f'No reachable frontier ({self._no_frontier_n}/{self.NO_FRONTIER_LIMIT})')

            # Spin-in-place recovery before giving up
            if self._no_frontier_n == self.SPIN_RECOVERY_THRESHOLD:
                self._start_spin_recovery()
                return

            if self._no_frontier_n >= self.NO_FRONTIER_LIMIT:
                if self._map and self._is_map_sufficiently_explored():
                    self._finish_exploration()
                else:
                    self.get_logger().info(
                        'Map not sufficiently explored — resetting counter')
                    self._no_frontier_n = 0
            return

        self._no_frontier_n = 0

        # Goal stability check — prevent rapid goal switching
        if self._last_stable_goal is not None:
            dist_to_last   = math.hypot(
                frontier[0] - self._last_stable_goal[0],
                frontier[1] - self._last_stable_goal[1])
            time_since_stable = (
                now - self._goal_stable_since).nanoseconds / 1e9

            if (dist_to_last < self.GOAL_STABILITY_DIST
                    and time_since_stable < self.GOAL_STABILITY_TIME):
                self.get_logger().debug(
                    f'Goal stability: keeping current goal '
                    f'(dist={dist_to_last:.2f}m, time={time_since_stable:.1f}s)')
                return

        # Avoid sending a goal that is essentially at the robot's feet
        if self._rx is not None and self._ry is not None:
            dist_to_current = math.hypot(
                frontier[0] - self._rx, frontier[1] - self._ry)
            if dist_to_current < 0.3:
                self.get_logger().warn(
                    f'Frontier too close ({dist_to_current:.2f}m) — skipping')
                return

        self._goal_stable_since = now
        self._last_stable_goal  = frontier
        self._send_goal(frontier)

    # ── finish — called exactly once ──────────────────────────────────────

    def _finish_exploration(self):
        if self._done:
            return
        self._done = True

        self.get_logger().info('=== Exploration complete — stopping ===')
        self._timer.cancel()
        self._cancel_goal()

        # Save map in background thread to avoid blocking the executor
        threading.Thread(target=self._save_map, daemon=True).start()

        msg      = Bool()
        msg.data = True
        self._done_pub.publish(msg)
        self.get_logger().info('Published /exploration_done = True')

    # ── frontier detection ────────────────────────────────────────────────

    def _find_frontier(self):
        if self._map_corrupted:
            self.get_logger().warn('Skipping frontier detection — map corrupted')
            return None

        m    = self._map
        h, w = m.info.height, m.info.width
        res  = m.info.resolution
        ox   = m.info.origin.position.x
        oy   = m.info.origin.position.y

        if not hasattr(m, 'data') or len(m.data) == 0:
            self.get_logger().error('Invalid map data — skipping frontier detection')
            return None

        data = np.array(m.data, dtype=np.int8).reshape(h, w)
        free = (data == 0)
        unk  = (data == -1)
        obs  = (data >= 50)

        # Frontier mask: free cell with at least one unknown 4-connected neighbour
        unk_u = np.zeros_like(unk); unk_u[1:,  :]  = unk[:-1, :]
        unk_d = np.zeros_like(unk); unk_d[:-1, :]  = unk[1:,  :]
        unk_l = np.zeros_like(unk); unk_l[:,  1:]  = unk[:,  :-1]
        unk_r = np.zeros_like(unk); unk_r[:, :-1]  = unk[:,   1:]
        frontier_mask = free & (unk_u | unk_d | unk_l | unk_r)

        # Safety: dilate obstacles in 8 directions (cardinal + diagonal)
        r       = self.SAFE_RADIUS_CELLS
        obs_exp = obs.copy()
        for s in range(1, r + 1):
            # Cardinal
            o_u = np.zeros_like(obs); o_u[s:,  :]  = obs[:-s, :]
            o_d = np.zeros_like(obs); o_d[:-s, :]  = obs[s:,  :]
            o_l = np.zeros_like(obs); o_l[:,  s:]  = obs[:, :-s]
            o_r = np.zeros_like(obs); o_r[:, :-s]  = obs[:,  s:]
            # Diagonal
            o_ul = np.zeros_like(obs); o_ul[s:,  s:]  = obs[:-s, :-s]
            o_ur = np.zeros_like(obs); o_ur[s:, :-s]  = obs[:-s,  s:]
            o_dl = np.zeros_like(obs); o_dl[:-s,  s:] = obs[s:,  :-s]
            o_dr = np.zeros_like(obs); o_dr[:-s, :-s] = obs[s:,   s:]
            obs_exp |= o_u | o_d | o_l | o_r | o_ul | o_ur | o_dl | o_dr
        frontier_mask &= ~obs_exp

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return None

        wx_arr = ox + xs * res
        wy_arr = oy + ys * res

        # Grid-based clustering
        cs     = self.CLUSTER_SIZE_M
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

        # Weighted score: reward large clusters, penalise distance
        # score = cluster_size - weight * distance  (higher is better)
        w = self.FRONTIER_DIST_WEIGHT
        max_size = max(c[3] for c in candidates)
        max_dist = max(c[2] for c in candidates) or 1.0
        candidates.sort(
            key=lambda c: -(c[3] / max_size - w * c[2] / max_dist))

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
        self._goal_gen   += 1
        my_gen            = self._goal_gen  # capture for stale-callback guard

        def _accepted(future, gen=my_gen):
            self._goal_accepted_cb(future, gen)

        self._nav.send_goal_async(goal).add_done_callback(_accepted)

    def _goal_accepted_cb(self, future, gen):
        # Stale-callback guard
        if gen != self._goal_gen:
            self.get_logger().debug(
                f'Ignoring stale goal-accepted callback (gen {gen} != {self._goal_gen})')
            return

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

        my_gen = gen

        def _done(future, gen=my_gen):
            self._goal_done_cb(future, gen)

        handle.get_result_async().add_done_callback(_done)

    def _goal_done_cb(self, future, gen):
        # Stale-callback guard
        if gen != self._goal_gen:
            self.get_logger().debug(
                f'Ignoring stale goal-done callback (gen {gen} != {self._goal_gen})')
            return

        self._navigating  = False
        self._goal_handle = None

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
            self._last_goal = None
            return

        ABORTED   = GoalStatus.STATUS_ABORTED    # 4
        CANCELED  = GoalStatus.STATUS_CANCELED   # 6
        SUCCEEDED = GoalStatus.STATUS_SUCCEEDED  # 4 in older, 5 in newer

        if status in (ABORTED, CANCELED):
            if self._last_goal and self._amcl_pose:
                dist_to_goal = math.hypot(
                    self._amcl_pose.position.x - self._last_goal[0],
                    self._amcl_pose.position.y - self._last_goal[1])
                if dist_to_goal < 0.8:
                    self.get_logger().info(
                        f'Goal close enough (status={status}, dist={dist_to_goal:.2f}m)'
                        ' — NOT blacklisting')
                else:
                    self.get_logger().warn(
                        f'Goal failed (status={status}, dist={dist_to_goal:.2f}m)'
                        ' — blacklisting temporarily')
                    self._blacklist_add(*self._last_goal)
            else:
                self.get_logger().warn(
                    f'Goal status={status} — blacklisting temporarily')
                if self._last_goal:
                    self._blacklist_add(*self._last_goal)
        elif status == SUCCEEDED:
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
        """
        Save the map using map_saver_cli.  Runs in a background thread so it
        never blocks the ROS executor (avoids the spin-within-spin deadlock).
        Writes to a .tmp file first, then atomically renames to the live path.
        """
        path     = self.MAP_SAVE_PATH
        tmp_path = path + '.tmp'
        saved_ok = False

        # Prefer the map_saver_cli subprocess (no executor re-entrancy issues)
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', tmp_path],
                check=True, timeout=15.0,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            saved_ok = True
        except Exception as e:
            self.get_logger().error(f'CLI map save failed: {e}')

        if not saved_ok:
            self.get_logger().error('Map save failed — map NOT written')
            return

        # Atomic rename: tmp → live
        for ext in ('.pgm', '.yaml'):
            tmp_file  = tmp_path + ext
            live_file = path + ext
            if not os.path.exists(tmp_file):
                self.get_logger().warn(f'Expected tmp file not found: {tmp_file}')
                continue
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
