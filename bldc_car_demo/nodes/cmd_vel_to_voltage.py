import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToVoltage(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_voltage')

        # --- Parametreler ---
        self.declare_parameter('wheel_separation', 0.36)     # [m]
        self.declare_parameter('wheel_radius', 0.08)         # [m]
        self.declare_parameter('max_voltage', 24.0)          # [V]
        self.declare_parameter('kv_vel', 0.04)               # [V / (rad/s)]
        self.declare_parameter('ks_static', 0.6)             # [V] statik sürtünme önyargısı
        self.declare_parameter('deadman_timeout', 0.5)       # [s]
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('left_cmd_topic', '/rear_left/voltage_cmd')
        self.declare_parameter('right_cmd_topic', '/rear_right/voltage_cmd')

        # === Yeni “sigorta”lar ===
        self.declare_parameter('yaw_sign', 1.0)              # +1: olduğu gibi, -1: angular.z ters
        self.declare_parameter('left_sign', 1.0)             # +1/-1: sol teker yön çevir
        self.declare_parameter('right_sign', 1.0)            # +1/-1: sağ teker yön çevir
        self.declare_parameter('swap_sides', False)          # True: sol/sağ komutları çaprazla
        self.declare_parameter('log_debug', False)           # True: periyodik log

        # Param okuma
        self.L = float(self.get_parameter('wheel_separation').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.VMAX = float(self.get_parameter('max_voltage').value)
        self.kv = float(self.get_parameter('kv_vel').value)
        self.ks = float(self.get_parameter('ks_static').value)
        self.deadman_timeout = float(self.get_parameter('deadman_timeout').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        left_topic = str(self.get_parameter('left_cmd_topic').value)
        right_topic = str(self.get_parameter('right_cmd_topic').value)

        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.left_sign = float(self.get_parameter('left_sign').value)
        self.right_sign = float(self.get_parameter('right_sign').value)
        self.swap_sides = bool(self.get_parameter('swap_sides').value)
        self.log_debug = bool(self.get_parameter('log_debug').value)

        # Publisherlar
        self.left_pub = self.create_publisher(Float64, left_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_topic, 10)

        # Subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        # Durum
        self.last_cmd = Twist()
        self.last_stamp = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(1.0, rate_hz), self.on_timer)
        self.get_logger().info(
            f'Mapping /cmd_vel -> {left_topic}, {right_topic} | '
            f'params: yaw_sign={self.yaw_sign}, left_sign={self.left_sign}, right_sign={self.right_sign}, swap={self.swap_sides}'
        )

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_stamp = self.get_clock().now()

    @staticmethod
    def _clamp(x: float, bound: float) -> float:
        return max(-bound, min(bound, x))

    def on_timer(self):
        now = self.get_clock().now()
        if (now - self.last_stamp).nanoseconds * 1e-9 > self.deadman_timeout:
            self.publish_voltage(0.0, 0.0)
            return

        vx = float(self.last_cmd.linear.x)       # ileri + (m/s)
        wz = float(self.last_cmd.angular.z)      # +: sola/CCW (REP-103)
        wz *= self.yaw_sign                      # Gerekirse işaret düzelt

        # Diferansiyel sürüş (teker açısal hızları, rad/s)
        omega_left  = (vx - 0.5 * wz * self.L) / self.R
        omega_right = (vx + 0.5 * wz * self.L) / self.R

        # Voltaj tahmini + statik sürtünme (simetrik)
        v_left  = self.kv * omega_left  + (self.ks if omega_left  > 0 else (-self.ks if omega_left  < 0 else 0.0))
        v_right = self.kv * omega_right + (self.ks if omega_right > 0 else (-self.ks if omega_right < 0 else 0.0))

        # Tek tek yön çevir (gerekirse)
        v_left  *= self.left_sign
        v_right *= self.right_sign

        # Satürasyon (simetrik)
        v_left  = self._clamp(v_left,  self.VMAX)
        v_right = self._clamp(v_right, self.VMAX)

        # Sol/Sağ’ı çaprazlamak gerekirse
        if self.swap_sides:
            v_left, v_right = v_right, v_left

        if self.log_debug:
            self.get_logger().info(f"vx={vx:.3f} wz={wz:.3f}  ->  vL={v_left:.3f}  vR={v_right:.3f}")

        self.publish_voltage(v_left, v_right)

    def publish_voltage(self, left_v: float, right_v: float):
        ml = Float64(); ml.data = float(left_v)
        mr = Float64(); mr.data = float(right_v)
        self.left_pub.publish(ml)
        self.right_pub.publish(mr)


def main():
    rclpy.init()
    node = CmdVelToVoltage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
