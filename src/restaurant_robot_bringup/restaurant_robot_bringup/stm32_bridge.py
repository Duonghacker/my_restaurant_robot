#!/usr/bin/env python3
"""
STM32 Bridge Node - Restaurant Robot Bringup
============================================
Giao tiếp 2 chiều giữa ROS 2 và STM32F401 qua UART:
  - Đọc telemetry ENC,... → tính odometry → publish /odom + TF odom→base_link
  - Subscribe /cmd_vel → tính inverse kinematics → gửi V,L,R lệnh xuống STM32

Thông số phần cứng (từ firmware f401_test_hc05):
  WHEEL_DIAMETER    = 0.065 m
  WHEEL_BASE        = 0.230 m
  LEFT_CPR          = 1510.5 counts/rev
  RIGHT_CPR         = 1484.75 counts/rev
  TELEMETRY_PERIOD  = 50 ms (20 Hz)
  UART              = USART1 (GPIO 14/15 → /dev/ttyAMA0), 115200 baud 8N1

Giao thức:
  STM32 → Pi: ENC,<tick_ms>,<l_tot>,<r_tot>,<l_rpm×10>,<r_rpm×10>,<l_mms>,<r_mms>\\r\\n
  Pi → STM32: V,<left_mm_s>,<right_mm_s>\\r\\n
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# ─────────────────────────────────────────────────
# Robot thông số (khớp chính xác với firmware)
# ─────────────────────────────────────────────────
WHEEL_DIAMETER_M         = 0.065
WHEEL_BASE_M             = 0.230
WHEEL_CIRCUMFERENCE_M    = math.pi * WHEEL_DIAMETER_M
LEFT_COUNTS_PER_REV      = 1510.5
RIGHT_COUNTS_PER_REV     = 1484.75

# Giới hạn tốc độ bánh (firmware MAX_WHEEL_RPM=170)
MAX_WHEEL_MM_S           = (170.0 * WHEEL_CIRCUMFERENCE_M * 1000.0) / 60.0  # ≈ 580 mm/s
CMD_TIMEOUT_STM32_MS     = 500   # STM32 tự dừng nếu không nhận lệnh trong 500ms

# ─────────────────────────────────────────────────
# QoS sensor-like (Best Effort, 10 depth)
# ─────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class STM32Bridge(Node):
    """
    ROS 2 node để giao tiếp với STM32F401.
    """

    def __init__(self):
        super().__init__('stm32_bridge')

        # ── Parameters ──────────────────────────────
        self.declare_parameter('serial_port',  '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('publish_tf',   True)
        self.declare_parameter('max_linear_mms',  350.0)   # mm/s an toàn trong nhà
        self.declare_parameter('max_angular_rads', 1.2)    # rad/s

        self.serial_port  = self.get_parameter('serial_port').value
        self.baud_rate    = self.get_parameter('baud_rate').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.publish_tf   = self.get_parameter('publish_tf').value
        self.max_linear_mms  = self.get_parameter('max_linear_mms').value
        self.max_angular_rads = self.get_parameter('max_angular_rads').value

        # ── State odometry ──────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.th  = 0.0
        self.vx  = 0.0  # linear velocity m/s (từ telemetry)
        self.wz  = 0.0  # angular velocity rad/s

        self.left_total_prev  = None   # None = chưa nhận packet đầu tiên
        self.right_total_prev = None
        self.last_enc_time    = None

        # ── Serial ──────────────────────────────────
        self.ser = None
        self._serial_lock = threading.Lock()
        self._open_serial()

        # ── Publishers ──────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ─────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )

        # ── Serial read thread ───────────────────────
        self._running = True
        self._read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True
        )
        self._read_thread.start()

        # ── Keepalive timer: gửi V,0,0 nếu không có cmd_vel ─
        self._last_cmd_time = time.time()
        self.create_timer(0.1, self._keepalive_cb)  # 10Hz check

        self.get_logger().info(
            f'STM32 Bridge started | port={self.serial_port} | '
            f'base={self.base_frame} | odom={self.odom_frame}'
        )

    # ════════════════════════════════════════════════
    # Serial connection
    # ════════════════════════════════════════════════

    def _open_serial(self):
        """Mở serial port, retry nếu thất bại."""
        while rclpy.ok():
            try:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.5,
                    write_timeout=0.5,
                )
                # Flush buffer cũ
                self.ser.reset_input_buffer()
                # Báo STM32 chuyển sang chế độ PI
                self._serial_send('M,PI\r\n')
                self.get_logger().info(
                    f'Serial opened: {self.serial_port} @ {self.baud_rate}'
                )
                return
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Cannot open {self.serial_port}: {e} — retry in 2s'
                )
                time.sleep(2.0)

    def _serial_send(self, text: str):
        """Thread-safe gửi text xuống STM32."""
        if self.ser and self.ser.is_open:
            with self._serial_lock:
                try:
                    self.ser.write(text.encode('ascii'))
                except serial.SerialException as e:
                    self.get_logger().warn(f'Serial write error: {e}')

    # ════════════════════════════════════════════════
    # Serial read loop (thread riêng)
    # ════════════════════════════════════════════════

    def _serial_read_loop(self):
        """Đọc liên tục từ STM32, parse line ENC,..."""
        while self._running and rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.5)
                continue
            try:
                with self._serial_lock:
                    raw = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.5)
                continue

            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if line.startswith('ENC,'):
                self._process_enc_line(line)

    # ════════════════════════════════════════════════
    # Parse ENC và tính odometry
    # ════════════════════════════════════════════════

    def _process_enc_line(self, line: str):
        """
        Format: ENC,<tick_ms>,<l_tot>,<r_tot>,<l_rpm×10>,<r_rpm×10>,<l_mms>,<r_mms>
        """
        parts = line.split(',')
        if len(parts) < 8:
            return

        try:
            tick_ms     = int(parts[1])
            left_total  = int(parts[2])
            right_total = int(parts[3])
            l_rpm_x10   = int(parts[4])
            r_rpm_x10   = int(parts[5])
            left_mms    = int(parts[6])
            right_mms   = int(parts[7])
        except (ValueError, IndexError):
            return

        now = self.get_clock().now()

        # ── Khởi tạo lần đầu ──────────────────────
        if self.left_total_prev is None:
            self.left_total_prev  = left_total
            self.right_total_prev = right_total
            self.last_enc_time    = now
            return

        # ── Delta counts ───────────────────────────
        dl_counts = left_total  - self.left_total_prev
        dr_counts = right_total - self.right_total_prev
        self.left_total_prev  = left_total
        self.right_total_prev = right_total

        # Sanity check: bỏ qua nếu delta quá lớn (reset encoder Z)
        MAX_DELTA = 5000  # counts/cycle max reasonable
        if abs(dl_counts) > MAX_DELTA or abs(dr_counts) > MAX_DELTA:
            self.get_logger().warn(
                f'Encoder delta too large ({dl_counts},{dr_counts}), skipping'
            )
            return

        # ── Delta distance (m) ─────────────────────
        dl_m = (dl_counts / LEFT_COUNTS_PER_REV)  * WHEEL_CIRCUMFERENCE_M
        dr_m = (dr_counts / RIGHT_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M

        # ── Differential drive kinematics ──────────
        d_center = (dl_m + dr_m) / 2.0
        d_theta  = (dr_m - dl_m) / WHEEL_BASE_M

        # Mid-point integration (chính xác hơn Euler)
        half_dth = d_theta / 2.0
        self.x  += d_center * math.cos(self.th + half_dth)
        self.y  += d_center * math.sin(self.th + half_dth)
        self.th += d_theta

        # Normalize theta về [-π, π]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # ── Velocities từ STM32 telemetry ──────────
        self.vx = ((left_mms + right_mms) / 2.0) / 1000.0        # m/s
        self.wz = ((right_mms - left_mms) / 1000.0) / WHEEL_BASE_M  # rad/s

        # ── Publish odometry ───────────────────────
        self._publish_odom(now)
        self.last_enc_time = now

    # ════════════════════════════════════════════════
    # Publish /odom và TF
    # ════════════════════════════════════════════════

    def _publish_odom(self, stamp):
        """Publish nav_msgs/Odometry và TF odom→base_link."""

        # Quaternion từ yaw (chỉ xoay quanh Z)
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # ── Odometry message ────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.wz

        # Covariance: diagonal (x,y,z,rx,ry,rz)
        # Encoder khá tin cậy, dùng giá trị nhỏ
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 1e6    # z (không dùng)
        odom.pose.covariance[21] = 1e6    # roll
        odom.pose.covariance[28] = 1e6    # pitch
        odom.pose.covariance[35] = 0.03   # yaw

        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[7]  = 1e6   # vy
        odom.twist.covariance[14] = 1e6   # vz
        odom.twist.covariance[21] = 1e6   # wx
        odom.twist.covariance[28] = 1e6   # wy
        odom.twist.covariance[35] = 0.05  # wz

        self.odom_pub.publish(odom)

        # ── TF odom → base_link ─────────────────────
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)

    # ════════════════════════════════════════════════
    # cmd_vel callback → inverse kinematics → STM32
    # ════════════════════════════════════════════════

    def _cmd_vel_cb(self, msg: Twist):
        """
        Nhận /cmd_vel, tính tốc độ từng bánh, gửi V,L,R xuống STM32.
        Inverse kinematics differential drive:
          v_left  = vx - omega * (WHEEL_BASE/2)
          v_right = vx + omega * (WHEEL_BASE/2)
        """
        vx    = float(msg.linear.x)   # m/s
        omega = float(msg.angular.z)  # rad/s

        # Clamp linear/angular theo giới hạn an toàn
        max_vx = self.max_linear_mms / 1000.0
        vx    = max(-max_vx, min(max_vx, vx))
        omega = max(-self.max_angular_rads, min(self.max_angular_rads, omega))

        # Inverse kinematics → mm/s
        half_base = WHEEL_BASE_M / 2.0
        left_mms  = (vx - omega * half_base) * 1000.0
        right_mms = (vx + omega * half_base) * 1000.0

        # Clamp theo max tốc độ vật lý của bánh
        left_mms  = max(-MAX_WHEEL_MM_S, min(MAX_WHEEL_MM_S, left_mms))
        right_mms = max(-MAX_WHEEL_MM_S, min(MAX_WHEEL_MM_S, right_mms))

        # Gửi xuống STM32
        cmd = f'V,{int(left_mms)},{int(right_mms)}\r\n'
        self._serial_send(cmd)
        self._last_cmd_time = time.time()

    # ════════════════════════════════════════════════
    # Keepalive: tránh STM32 timeout 500ms
    # ════════════════════════════════════════════════

    def _keepalive_cb(self):
        """
        Nếu không có cmd_vel trong 400ms → gửi V,0,0 để giữ kết nối.
        (STM32 timeout là 500ms → dừng nếu không nhận lệnh)
        """
        if time.time() - self._last_cmd_time > 0.4:
            self._serial_send('V,0,0\r\n')
            self._last_cmd_time = time.time()

    # ════════════════════════════════════════════════
    # Cleanup
    # ════════════════════════════════════════════════

    def destroy_node(self):
        self._running = False
        self._serial_send('S\r\n')   # dừng robot khi node tắt
        time.sleep(0.1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
