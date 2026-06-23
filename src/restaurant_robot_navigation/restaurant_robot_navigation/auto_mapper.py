#!/usr/bin/env python3
"""
auto_explorer.py
────────────────────────────────────────────────────────────────────────────
Robot restaurant — Autonomous Exploration for Nav2 map building
ROS 2 Jazzy | slam_toolbox (online_async)

Architecture:
  ┌─────────────────────────────────────────────────────────┐
  │  OccupancyGrid (/map)  ──►  FrontierDetector            │
  │  LaserScan (/scan)     ──►  ObstacleGuard               │
  │                                                         │
  │  FrontierDetector  ──►  GoalSelector  ──►  Direct Drive │
  │  ObstacleGuard     ──►  RecoveryManager                 │
  └─────────────────────────────────────────────────────────┘

Robot specs (từ robot_core.xacro):
  chassis : 0.30 × 0.30 × 0.15 m
  wheel_separation : 0.35 m  → half_width ≈ 0.20 m
  wheel_radius     : 0.05 m

World (obs.sdf):
  20 × 15 m, có kitchen_partition, 6 bàn + ghế
"""

import math
import time
import random
from enum import Enum, auto
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# ═══════════════════════════════════════════════════════════════════════════
# CONSTANTS — căn chỉnh theo robot_core.xacro + obs.sdf
# ═══════════════════════════════════════════════════════════════════════════

# Robot geometry
ROBOT_HALF_WIDTH  = 0.20   # m — thêm safety margin 5 cm
ROBOT_HALF_LENGTH = 0.17   # m

# Obstacle thresholds
DANGER_DIST  = 0.28   # m — lùi ngay
STOP_DIST    = 0.45   # m — dừng, trigger recovery
CAUTION_DIST = 0.65   # m — giảm tốc

# Navigation speeds
SPEED_FWD   =  0.40   # m/s
SPEED_BACK  = -0.12   # m/s
SPEED_TURN  =  0.65   # rad/s

# Frontier detection
MIN_FRONTIER_SIZE    = 8    # cells — bỏ qua frontier quá nhỏ
FRONTIER_SAMPLE_STEP = 3    # bước nhảy khi scan map (tiết kiệm CPU)
MIN_GOAL_DISTANCE    = 0.8  # m — không chọn goal quá gần robot
MAX_GOAL_DISTANCE    = 9.0  # m — world 20×15, không vươn quá xa 1 lần
GOAL_REACHED_RADIUS  = 0.50 # m — coi như đã tới đích

# Timeouts
DRIVE_TIMEOUT_SEC    = 45.0  # giây — cancel nếu bị kẹt lâu
RECOVERY_TIMEOUT_SEC = 8.0   # giây — thời gian tối đa của 1 recovery step
STUCK_CHECK_INTERVAL = 5.0   # giây — kiểm tra robot có di chuyển không
STUCK_MOVE_THRESHOLD = 0.05  # m — coi là stuck nếu di chuyển < 5 cm

# Exploration
EXPLORE_COVERAGE_THRESHOLD = 0.85  # 85% map đã known → dừng
RANDOM_GOAL_FALLBACK_TRIES = 5     # thử random goal khi không có frontier


class ExploreState(Enum):
    INIT          = auto()
    DETECTING     = auto()   # đang phân tích frontier
    DRIVING       = auto()   # đang tự lái tới đích
    WALL_FOLLOWING= auto()   # Bám tường thoát kẹt
    RECOVERY      = auto()   # đang thực hiện recovery
    STUCK         = auto()   # bị kẹt, cần rotate escape
    DONE          = auto()   # map đã cover xong


# ═══════════════════════════════════════════════════════════════════════════
class FrontierDetector:
    """
    Phân tích OccupancyGrid để tìm frontier cells.
    Frontier = ô FREE (-1 < val < 50) kề cạnh ô UNKNOWN (val == -1).
    """

    def __init__(self, node: Node):
        self._node = node
        self._map: Optional[OccupancyGrid] = None

    def update_map(self, msg: OccupancyGrid):
        self._map = msg

    def has_map(self) -> bool:
        return self._map is not None

    def get_coverage(self) -> float:
        """Trả về tỉ lệ ô đã biết (known / total)."""
        if self._map is None:
            return 0.0
        data = self._map.data
        total = len(data)
        unknown = sum(1 for v in data if v == -1)
        return (total - unknown) / max(total, 1)

    def detect_frontiers(self) -> List[Tuple[float, float]]:
        """
        Trả về danh sách (x, y) world-frame của các frontier centroids.
        Dùng BFS-style connected-component để gom frontier cells thành cụm.
        """
        if self._map is None:
            return []

        grid   = self._map
        width  = grid.info.width
        height = grid.info.height
        res    = grid.info.resolution
        ox     = grid.info.origin.position.x
        oy     = grid.info.origin.position.y
        data   = np.array(grid.data, dtype=np.int8).reshape((height, width))

        # ── 1. Tìm tất cả frontier cells ──────────────────────────────
        free_mask    = (data >= 0) & (data < 50)   # known free
        unknown_mask = (data == -1)                 # unknown

        # Pad để tránh out-of-bounds khi check neighbours
        unknown_padded = np.pad(unknown_mask, 1, constant_values=False)

        # Frontier: free cell có ít nhất 1 neighbour unknown
        frontier_mask = np.zeros_like(free_mask, dtype=bool)
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            shifted = unknown_padded[1+dr:height+1+dr, 1+dc:width+1+dc]
            frontier_mask |= (free_mask & shifted)

        # ── 2. Cluster frontier cells bằng connected-components ───────
        from collections import deque
        visited   = np.zeros_like(frontier_mask, dtype=bool)
        centroids = []

        rows, cols = np.where(frontier_mask)
        for start_r, start_c in zip(rows[::FRONTIER_SAMPLE_STEP],
                                    cols[::FRONTIER_SAMPLE_STEP]):
            if visited[start_r, start_c]:
                continue
            # BFS
            queue   = deque([(start_r, start_c)])
            cluster = []
            while queue:
                r, c = queue.popleft()
                if r < 0 or r >= height or c < 0 or c >= width:
                    continue
                if visited[r, c] or not frontier_mask[r, c]:
                    continue
                visited[r, c] = True
                cluster.append((r, c))
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    queue.append((r+dr, c+dc))

            if len(cluster) < MIN_FRONTIER_SIZE:
                continue

            # Centroid → world frame
            mean_r = sum(p[0] for p in cluster) / len(cluster)
            mean_c = sum(p[1] for p in cluster) / len(cluster)
            wx = ox + (mean_c + 0.5) * res
            wy = oy + (mean_r + 0.5) * res
            centroids.append((wx, wy))

        return centroids


# ═══════════════════════════════════════════════════════════════════════════
class RecoveryManager:
    """
    Quản lý các bước recovery khi robot bị chặn hoặc stuck.
    Sequence: backup → rotate_random → backup_slow → rotate_more
    """

    RECOVERY_STEPS = [
        ("backup",       -0.12, 0.0,   2.0),   # (name, linear, angular, duration)
        ("rotate_right",  0.0, -0.80,  2.5),
        ("backup",       -0.10, 0.0,   1.5),
        ("rotate_left",   0.0,  0.80,  3.0),
        ("forward_slow",  0.20, 0.0,   1.0),
        ("rotate_random", 0.0,  0.0,   3.0),   # angular được set ngẫu nhiên
    ]

    def __init__(self, cmd_pub):
        self._pub       = cmd_pub
        self._step_idx  = 0
        self._start_time: Optional[float] = None

    def reset(self):
        self._step_idx  = 0
        self._start_time = None

    def is_done(self) -> bool:
        return self._step_idx >= len(self.RECOVERY_STEPS)

    def execute_step(self, now: float) -> bool:
        """
        Gọi liên tục trong timer. Trả về True khi bước hiện tại xong.
        """
        if self.is_done():
            return True

        name, lin, ang, dur = self.RECOVERY_STEPS[self._step_idx]

        if self._start_time is None:
            self._start_time = now
            # Xoay ngẫu nhiên
            if name == "rotate_random":
                ang = random.choice([-0.9, 0.9])
                self.RECOVERY_STEPS[self._step_idx] = (name, lin, ang, dur)

        twist = Twist()
        twist.linear.x  = lin
        twist.angular.z = ang
        self._pub.publish(twist)

        if now - self._start_time >= dur:
            # Stop
            self._pub.publish(Twist())
            self._step_idx  += 1
            self._start_time = None
            return True
        return False


# ═══════════════════════════════════════════════════════════════════════════
class ObstacleGuard:
    """
    Đọc LaserScan, phát hiện obstacle ở 5 sector.
    Tách riêng khỏi state-machine để dễ test.
    """

    def __init__(self):
        self.front      = 12.0
        self.front_left = 12.0
        self.front_right= 12.0
        self.left       = 12.0
        self.right      = 12.0

    def update(self, msg: LaserScan):
        r  = msg.ranges
        ai = msg.angle_min
        inc= msg.angle_increment

        self.front       = self._sector(r, ai, inc,   0.0, 22)
        self.front_left  = self._sector(r, ai, inc,  40.0, 18)
        self.front_right = self._sector(r, ai, inc, -40.0, 18)
        self.left        = self._sector(r, ai, inc,  90.0, 18)
        self.right       = self._sector(r, ai, inc, -90.0, 18)

    @staticmethod
    def _sector(ranges, angle_min, angle_inc, center_deg, half_deg) -> float:
        n    = len(ranges)
        c_r  = math.radians(center_deg)
        h_r  = math.radians(half_deg)
        ic   = int((c_r - angle_min) / angle_inc)
        ih   = int(h_r / angle_inc)
        vals = [
            ranges[i % n]
            for i in range(ic - ih, ic + ih + 1)
            if math.isfinite(ranges[i % n]) and 0.08 < ranges[i % n] < 12.0
        ]
        return min(vals) if vals else 12.0

    @property
    def immediate_danger(self) -> bool:
        return self.front < DANGER_DIST

    @property
    def should_stop(self) -> bool:
        return (self.front < STOP_DIST or
                self.front_left  < STOP_DIST * 0.85 or
                self.front_right < STOP_DIST * 0.85)


# ═══════════════════════════════════════════════════════════════════════════
class AutoExplorer(Node):
    """
    Node chính: điều phối frontier exploration + Direct Driving (tf2).
    
    Flow:
      INIT → (map/tf ready?) → DETECTING → (frontier found?) → DRIVING
         ↑                                    ↓ (timeout)
         └─────────── RECOVERY ←── STUCK ←────┘
                                ↓ (done)
                              DONE
    """

    def __init__(self):
        super().__init__('auto_explorer')

        # ── Publishers / Subscribers ─────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/map',
                                 self._map_cb, qos_map)

        # ── TF2 Buffer / Listener (thay cho amcl_pose) ────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Sub-systems ───────────────────────────────────────────────
        self._frontier  = FrontierDetector(self)
        self._guard     = ObstacleGuard()
        self._recovery  = RecoveryManager(self._cmd_pub)

        # ── State ─────────────────────────────────────────────────────
        self._state          = ExploreState.INIT
        self._robot_x        = 0.0
        self._robot_y        = 0.0
        self._robot_yaw      = 0.0
        self._last_x         = 0.0
        self._last_y         = 0.0
        self._last_move_time = time.time()
        self._drive_start_time= 0.0
        self._current_goal   : Optional[Tuple[float,float]] = None
        self._visited_goals  : List[Tuple[float,float]] = []  # đã đến rồi
        self._fail_count     = 0  # số lần liên tiếp không tìm được frontier
        self._wall_follow_dir = 1.0 # 1.0 = bám tường phải, -1.0 = bám tường trái
        self._wall_follow_start_dist = 0.0

        # ── Main timer 10 Hz ──────────────────────────────────────────
        self.create_timer(0.10, self._tick)

        self.get_logger().info(
            '🤖 AutoExplorer (Direct Drive) khởi động | '
            f'danger={DANGER_DIST}m stop={STOP_DIST}m '
            f'frontier_min={MIN_FRONTIER_SIZE}cells')

    # ────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._guard.update(msg)

    def _map_cb(self, msg: OccupancyGrid):
        self._frontier.update_map(msg)

    def _update_pose_from_tf(self):
        """Cập nhật vị trí robot từ tf (map -> base_footprint)."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self._robot_x = trans.transform.translation.x
            self._robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._robot_yaw = math.atan2(siny, cosy)
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    # ────────────────────────────────────────────────────────────────
    # Main tick
    # ────────────────────────────────────────────────────────────────

    def _tick(self):
        now = time.time()

        # Update pose first
        if self._state != ExploreState.INIT:
            self._update_pose_from_tf()

        # ── Luôn kiểm tra obstacle khẩn cấp ────────────────────────
        if self._guard.immediate_danger and self._state != ExploreState.RECOVERY:
            self.get_logger().warn(
                f'⚠️  DANGER! front={self._guard.front:.2f}m → RECOVERY')
            self._cmd_pub.publish(Twist())
            self._recovery.reset()
            self._state = ExploreState.RECOVERY

        # ── Dispatch ────────────────────────────────────────────────
        if   self._state == ExploreState.INIT:
            self._state_init()
        elif self._state == ExploreState.DETECTING:
            self._state_detecting()
        elif self._state == ExploreState.DRIVING:
            self._state_driving(now)
        elif self._state == ExploreState.WALL_FOLLOWING:
            self._state_wall_following(now)
        elif self._state == ExploreState.RECOVERY:
            self._state_recovery(now)
        elif self._state == ExploreState.STUCK:
            self._state_stuck(now)
        elif self._state == ExploreState.DONE:
            self._state_done()

    # ────────────────────────────────────────────────────────────────
    # State handlers
    # ────────────────────────────────────────────────────────────────

    def _state_init(self):
        if not self._frontier.has_map():
            self.get_logger().info('⏳ Chờ /map từ slam_toolbox...', throttle_duration_sec=5.0)
            return
        if not self._update_pose_from_tf():
            self.get_logger().info('⏳ Chờ TF map -> base_footprint...', throttle_duration_sec=5.0)
            return
        self.get_logger().info('✅ Map + TF sẵn sàng → DETECTING')
        self._state = ExploreState.DETECTING

    def _state_detecting(self):
        # Kiểm tra coverage
        cov = self._frontier.get_coverage()
        if cov >= EXPLORE_COVERAGE_THRESHOLD:
            self.get_logger().info(
                f'🎉 Map đã cover {cov*100:.1f}% → DONE')
            self._state = ExploreState.DONE
            return

        frontiers = self._frontier.detect_frontiers()
        if not frontiers:
            self._fail_count += 1
            if self._fail_count > 3:
                self.get_logger().warn('Không tìm thấy frontier → random goal')
                goal = self._random_goal()
                if goal:
                    self._drive_to_goal(*goal)
            return
        self._fail_count = 0

        # Chọn frontier tốt nhất
        goal = self._select_best_frontier(frontiers)
        if goal is None:
            self.get_logger().warn('Tất cả frontier đều không phù hợp', throttle_duration_sec=5.0)
            return

        self.get_logger().info(
            f'🔍 Frontier: {len(frontiers)} cụm | '
            f'Chọn ({goal[0]:.2f}, {goal[1]:.2f}) | '
            f'Cover={cov*100:.1f}%')
        self._drive_to_goal(*goal)

    def _state_driving(self, now: float):
        if not self._current_goal:
            self._state = ExploreState.DETECTING
            return

        gx, gy = self._current_goal

        # Tính khoảng cách
        dx = gx - self._robot_x
        dy = gy - self._robot_y
        dist = math.hypot(dx, dy)

        if dist < GOAL_REACHED_RADIUS:
            self.get_logger().info(f'✅ Đến đích ({gx:.2f}, {gy:.2f})')
            self._visited_goals.append((gx, gy))
            self._cmd_pub.publish(Twist())
            self._state = ExploreState.DETECTING
            return

        # Timeout check
        elapsed = now - self._drive_start_time
        if elapsed > DRIVE_TIMEOUT_SEC:
            self.get_logger().warn(
                f'⏰ Drive timeout {elapsed:.0f}s → RECOVERY')
            self._cmd_pub.publish(Twist())
            self._recovery.reset()
            self._state = ExploreState.RECOVERY
            return

        # Stuck check
        moved_dx = self._robot_x - self._last_x
        moved_dy = self._robot_y - self._last_y
        moved = math.hypot(moved_dx, moved_dy)
        if now - self._last_move_time > STUCK_CHECK_INTERVAL:
            if moved < STUCK_MOVE_THRESHOLD:
                self.get_logger().warn(
                    f'🔒 Robot stuck! moved={moved:.3f}m → RECOVERY')
                self._cmd_pub.publish(Twist())
                self._recovery.reset()
                self._state = ExploreState.RECOVERY
                return
            self._last_x = self._robot_x
            self._last_y = self._robot_y
            self._last_move_time = now

        # Obstacle check - Nếu phía trước bị chặn, chuyển sang bám tường
        if self._guard.should_stop:
            self.get_logger().warn(f'🛑 Bị chặn phía trước ({self._guard.front:.2f}m) → WALL_FOLLOWING')
            self._cmd_pub.publish(Twist())
            # Quyết định bám tường trái hay phải: bên nào thoáng hơn thì quay sang đó, tường ở phía ngược lại
            if self._guard.left > self._guard.right:
                self._wall_follow_dir = -1.0 # Quay trái -> tường ở bên phải
            else:
                self._wall_follow_dir = 1.0  # Quay phải -> tường ở bên trái
            self._wall_follow_start_dist = dist
            self._state = ExploreState.WALL_FOLLOWING
            return

        # ── Điều khiển lái trực tiếp tới đích với APF (Lực đẩy) ──
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self._robot_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # Lực đẩy từ chướng ngại vật 2 bên để đi vào giữa khe hẹp
        repulsive_yaw = 0.0
        # Ngưỡng tác dụng lực: 0.6m
        if self._guard.left < 0.6:
            repulsive_yaw -= 0.8 * (0.6 - self._guard.left)  # Đẩy sang phải
        if self._guard.right < 0.6:
            repulsive_yaw += 0.8 * (0.6 - self._guard.right) # Đẩy sang trái
        
        # Kết hợp lực
        final_yaw_error = yaw_error + repulsive_yaw
        final_yaw_error = (final_yaw_error + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        # Nếu lệch hướng nhiều (> 0.5 rad ≈ 28 độ), xoay tại chỗ
        if abs(final_yaw_error) > 0.5:
            twist.linear.x = 0.0
            twist.angular.z = SPEED_TURN if final_yaw_error > 0 else -SPEED_TURN
            # Khi đang xoay tại chỗ, x và y không đổi -> Tránh bị nhận nhầm là stuck
            self._last_move_time = now
            self._last_x = self._robot_x
            self._last_y = self._robot_y
        else:
            # Vừa tiến vừa xoay điều chỉnh
            twist.linear.x = SPEED_FWD
            # Giảm tốc nếu thấy vật cản ở xa một chút
            if self._guard.front < CAUTION_DIST:
                twist.linear.x = SPEED_FWD * 0.5
            # Điều khiển P cho góc xoay
            twist.angular.z = final_yaw_error * 1.5
            # Giới hạn angular speed
            twist.angular.z = max(-SPEED_TURN, min(SPEED_TURN, twist.angular.z))

        self._cmd_pub.publish(twist)

    def _state_wall_following(self, now: float):
        if not self._current_goal:
            self._state = ExploreState.DETECTING
            return

        gx, gy = self._current_goal
        dx = gx - self._robot_x
        dy = gy - self._robot_y
        dist = math.hypot(dx, dy)

        # Điều kiện thoát WALL_FOLLOWING: Hướng tới đích thông thoáng VÀ (gần đích hơn lúc bắt đầu bám tường HOẶC đường siêu thoáng)
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self._robot_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        
        # Nếu đang xoay mặt về đích (chênh lệch góc nhỏ) và phía trước siêu thoáng
        if abs(yaw_error) < 0.3 and self._guard.front > 1.2 and self._guard.front_left > 1.0 and self._guard.front_right > 1.0:
            if dist < self._wall_follow_start_dist - 0.2:
                self.get_logger().info('🔓 Đường tới đích đã thoáng, thoát WALL_FOLLOWING → DRIVING')
                self._state = ExploreState.DRIVING
                return

        # Timeout check
        elapsed = now - self._drive_start_time
        if elapsed > DRIVE_TIMEOUT_SEC * 1.5: # Cho thêm thời gian khi bám tường
            self.get_logger().warn(
                f'⏰ Wall Follow timeout {elapsed:.0f}s → RECOVERY')
            self._cmd_pub.publish(Twist())
            self._recovery.reset()
            self._state = ExploreState.RECOVERY
            return

        # Cập nhật stuck
        moved_dx = self._robot_x - self._last_x
        moved_dy = self._robot_y - self._last_y
        moved = math.hypot(moved_dx, moved_dy)
        if now - self._last_move_time > STUCK_CHECK_INTERVAL:
            if moved < STUCK_MOVE_THRESHOLD:
                self.get_logger().warn(
                    f'🔒 Robot stuck trong lúc bám tường! moved={moved:.3f}m → RECOVERY')
                self._cmd_pub.publish(Twist())
                self._recovery.reset()
                self._state = ExploreState.RECOVERY
                return
            self._last_x = self._robot_x
            self._last_y = self._robot_y
            self._last_move_time = now

        # Điều khiển PID bám tường
        # Target distance to wall = 0.40m
        target_dist = 0.40
        twist = Twist()

        # Kiểm tra kẹt phía trước trong lúc bám tường
        if self._guard.front < DANGER_DIST:
            # Xoay tại chỗ
            twist.linear.x = 0.0
            twist.angular.z = -SPEED_TURN * self._wall_follow_dir
            self._last_move_time = now # Tránh bị kẹt do xoay
        elif self._guard.front < STOP_DIST:
            twist.linear.x = 0.0
            twist.angular.z = -SPEED_TURN * self._wall_follow_dir
            self._last_move_time = now
        else:
            # Đo khoảng cách tới tường
            if self._wall_follow_dir == 1.0:
                wall_dist = self._guard.left
                front_corner = self._guard.front_left
            else:
                wall_dist = self._guard.right
                front_corner = self._guard.front_right

            twist.linear.x = SPEED_FWD * 0.7 # Đi chậm hơn chút
            
            # Nếu góc vướng, cua gắt
            if front_corner < target_dist:
                twist.angular.z = -SPEED_TURN * self._wall_follow_dir
            # Nếu mất tường (wall_dist lớn), cua vào tìm tường
            elif wall_dist > target_dist + 0.15:
                twist.angular.z = SPEED_TURN * 0.8 * self._wall_follow_dir
            # Nếu gần tường, cua ra
            elif wall_dist < target_dist - 0.05:
                twist.angular.z = -SPEED_TURN * 0.8 * self._wall_follow_dir
            else:
                # Đi thẳng
                twist.angular.z = 0.0

        self._cmd_pub.publish(twist)

    def _state_recovery(self, now: float):
        done = self._recovery.execute_step(now)
        if self._recovery.is_done():
            self.get_logger().info('✅ Recovery xong → DETECTING')
            self._recovery.reset()
            self._state = ExploreState.DETECTING

    def _state_stuck(self, now: float):
        # Fallback rotate mạnh hơn
        twist = Twist()
        twist.angular.z = random.choice([-1.0, 1.0])
        self._cmd_pub.publish(twist)
        if now - self._drive_start_time > RECOVERY_TIMEOUT_SEC:
            self._cmd_pub.publish(Twist())
            self._state = ExploreState.DETECTING

    def _state_done(self):
        self._cmd_pub.publish(Twist())
        self.get_logger().info(
            '🗺️  Exploration hoàn tất! ',
            throttle_duration_sec=10.0)

    # ────────────────────────────────────────────────────────────────
    # Navigation helpers
    # ────────────────────────────────────────────────────────────────

    def _select_best_frontier(self,
                              frontiers: List[Tuple[float,float]]
                              ) -> Optional[Tuple[float,float]]:
        """
        Chấm điểm frontier theo:
          score = w_dist * (1/dist) + w_info * cluster_size
        → Ưu tiên frontier gần + nhiều unknown xung quanh.
        Loại bỏ goal đã từng đến và quá gần/xa.
        """
        best_score = -1.0
        best_goal  = None

        for fx, fy in frontiers:
            dist = math.hypot(fx - self._robot_x, fy - self._robot_y)
            if dist < MIN_GOAL_DISTANCE or dist > MAX_GOAL_DISTANCE:
                continue
            # Bỏ qua nếu đã đến rồi (trong bán kính 0.4 m)
            too_close_to_visited = any(
                math.hypot(fx - vx, fy - vy) < 0.4
                for vx, vy in self._visited_goals)
            if too_close_to_visited:
                continue

            # Score: frontier gần hơn → score cao hơn
            score = 1.0 / (dist + 0.1)

            if score > best_score:
                best_score = score
                best_goal  = (fx, fy)

        return best_goal

    def _random_goal(self) -> Optional[Tuple[float,float]]:
        """Sinh goal ngẫu nhiên trong world bounds khi không có frontier."""
        # World: 20×15, robot ở gần gốc tọa độ
        for _ in range(RANDOM_GOAL_FALLBACK_TRIES):
            rx = random.uniform(-8.5, 8.5)
            ry = random.uniform(-6.5, 6.5)
            dist = math.hypot(rx - self._robot_x, ry - self._robot_y)
            if MIN_GOAL_DISTANCE < dist < MAX_GOAL_DISTANCE:
                return rx, ry
        return None

    def _drive_to_goal(self, gx: float, gy: float):
        """Khởi tạo hành trình tự lái tới đích."""
        self._current_goal  = (gx, gy)
        self._drive_start_time = time.time()
        self._last_move_time= time.time()
        self._last_x        = self._robot_x
        self._last_y        = self._robot_y
        self._state         = ExploreState.DRIVING
        self.get_logger().info(f'🚀 Lái tới goal → ({gx:.2f}, {gy:.2f})')


# ═══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Dừng exploration theo yêu cầu.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()