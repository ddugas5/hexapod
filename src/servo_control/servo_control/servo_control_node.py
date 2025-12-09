import rclpy
from rclpy.node import node

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Float32Array,
            '/ik_angles',
            self.angle_callback,
            10
        )

    def angle_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)

    servo_control_node.destoy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
