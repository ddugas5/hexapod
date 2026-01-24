import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Float32,
            '/servo_angle',
            self.angle_callback,
            10
        )

        #set the number of channels on the board
        self.kit = ServoKit(channels=16)

        #set the servo channel to the channel you want
        self.servo_channel = 0

    def angle_callback(self, msg: Float32):
        angle = msg.data    #get data and assign to variable

        self.kit.servo[self.servo_channel].angle = angle #attach the specified channel to the servo and the recieved angle to the servo

        self.get_logger().info(f'Set servo to angle {angle:.1f} degrees')

def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)

    servo_control_node.destoy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()