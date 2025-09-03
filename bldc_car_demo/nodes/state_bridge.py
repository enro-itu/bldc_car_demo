import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Bu düğüm BLDC pluginlerinden gelen motor durumlarını tekrar yayınlar.
# Her pluginin Float64MultiArray ile en az [speed_rad_s, current_A, torque_Nm, voltage_V]
# bilgilerini yayınladığı varsayılır. Eğer plugin farklı mesaj kullanıyorsa,
# subscriber tiplerini ve publish logic'ini buna göre güncelleyin.

class StateBridge(Node):
    def __init__(self):
        super().__init__('state_bridge')

        # Parametreleri al
        self.declare_parameter('left_state_in', '/rear_left/state')
        self.declare_parameter('right_state_in', '/rear_right/state')
        self.declare_parameter('left_state_out', '/bldc_car/left/motor_state')
        self.declare_parameter('right_state_out', '/bldc_car/right/motor_state')

        self.left_in = self.get_parameter('left_state_in').get_parameter_value().string_value
        self.right_in = self.get_parameter('right_state_in').get_parameter_value().string_value
        self.left_out = self.get_parameter('left_state_out').get_parameter_value().string_value
        self.right_out = self.get_parameter('right_state_out').get_parameter_value().string_value

        # Publisherlar
        self.left_pub = self.create_publisher(Float64MultiArray, self.left_out, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, self.right_out, 10)

        # Subscriberlar
        self.left_sub = self.create_subscription(Float64MultiArray, self.left_in, self.on_left, 10)
        self.right_sub = self.create_subscription(Float64MultiArray, self.right_in, self.on_right, 10)

        self.get_logger().info(
            f'Republishing motor states: {self.left_in}->{self.left_out}, {self.right_in}->{self.right_out}'
        )

    def on_left(self, msg: Float64MultiArray):
        self.left_pub.publish(msg)

    def on_right(self, msg: Float64MultiArray):
        self.right_pub.publish(msg)


def main():
    rclpy.init()
    node = StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
