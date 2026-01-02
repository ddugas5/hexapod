import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class JointCommandNode(Node):
    def __init__(self):
        super().__init__('joint_command_node')
        #use this subscriber in the future
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ik_solver',
            self.ik_callback,
            10
        )

        #publisher to servo node
        self.servo_publisher = self.create_publisher(Float32MultiArray, '/leg/joint_angles', 10)

    def ik_callback(self, msg):
        self.servo_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    joint_command_node = JointCommandNode()

    rclpy.spin(joint_command_node)

    joint_command_node.destoy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()