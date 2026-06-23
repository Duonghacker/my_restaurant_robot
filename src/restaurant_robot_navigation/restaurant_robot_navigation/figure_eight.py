#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Thư viện để nhận góc hướng thực tế của robot

class FigureEightNode(Node):
    def __init__(self):
        super().__init__('figure_eight_node')

        # --- Khai báo tham số ---
        self.declare_parameter('linear_velocity', 0.5)  # Đã tăng tốc độ từ 0.2 lên 0.5 m/s
        self.declare_parameter('circle_radius', 0.5)    # Bán kính mỗi vòng tròn (m)
        self.declare_parameter('timer_period', 0.05)    # s (20 Hz)

        self.v = self.get_parameter('linear_velocity').value
        radius = self.get_parameter('circle_radius').value
        self.dt = self.get_parameter('timer_period').value

        # Tính w từ bán kính: w = v / R
        self.w = self.v / radius if radius > 0 else 0.0
        self.is_turning_left = True
        # Theo dõi góc yaw thực tế (rất quan trọng)
        self.current_yaw = 0.0
        self.previous_yaw = None
        self.accumulated_yaw = 0.0  # Tổng góc đã xoay được trong vòng tròn hiện tại
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # SUBSCRIBE TOPIC ODOMETRY ĐỂ LẤY GÓC THỰC TẾ
        # Lưu ý: Thay '/odom' bằng topic odometry của bạn trong mô phỏng (ví dụ: '/robot1/odom')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        self.timer = self.create_timer(self.dt, self._timer_callback)
        self.get_logger().info(
            f'Chuẩn bị vẽ số 8 | v={self.v:.2f} m/s | w={self.w:.2f} rad/s | R={radius} m'
        )
    def _odom_callback(self, msg: Odometry):
        """Hàm này được gọi mỗi khi có dữ liệu vị trí mới từ robot"""
        # Lấy góc Euler (Roll, Pitch, Yaw) từ Quaternion
        orientation_q = msg.pose.pose.orientation
        # Chuyển đổi Quaternion sang Yaw (góc xoay theo trục Z)
        self.current_yaw = self._euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        # Cộng dồn góc xoay mỗi lần nhận dữ liệu
        if self.previous_yaw is not None:
            delta_yaw = self._get_angle_diff(self.current_yaw, self.previous_yaw)
            self.accumulated_yaw += delta_yaw 
        self.previous_yaw = self.current_yaw

    def _euler_from_quaternion(self, x, y, z, w):
        """Hàm phụ trợ chuyển Quaternion sang góc Yaw"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4) # Kết quả trả về nằm trong khoảng [-pi, pi]
        return yaw_z

    def _get_angle_diff(self, target, current):
        """Tính toán độ chênh lệch góc an toàn (xử lý trường hợp vượt qua mốc +-pi)"""
        diff = target - current
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    def _timer_callback(self) -> None:
        # Kiểm tra xem tổng góc xoay (trị tuyệt đối) đã đạt 1 vòng tròn chưa (2 * Pi)
        if abs(self.accumulated_yaw) >= 2.0 * math.pi:
            self.is_turning_left = not self.is_turning_left
            self.accumulated_yaw = 0.0  # Reset lại bộ đếm góc cho vòng quay mới
            
            direction = 'TRÁI' if self.is_turning_left else 'PHẢI'
            self.get_logger().info(f'Hoàn thành 1 vòng, đổi hướng → {direction}')

        # Gửi lệnh vận hành
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w if self.is_turning_left else -self.w
        self.publisher_.publish(msg)

    def stop(self) -> None:
        self.publisher_.publish(Twist())
        self.get_logger().info('Robot đã dừng.')


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()