import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class TrajectoryControlNode(Node):
    def __init__(self):    
        super().__init__('trajectory_control_node')

        self.point_publisher = self.create_publisher(Float32MultiArray, '/foot_goal', 10)
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.matrix = [[1.0, 5.0, 0.0], [-1.0, 5.0, 0.0], [1.0, 5.0, 0.0], [-1.0, 5.0, 0.0], [1.0, 5.0, 0.0], [-1.0, 5.0, 0.0]]

    def timer_callback(self):
        msg = Float32MultiArray()

        msg.data = np.array(self.matrix).flatten().tolist()

        self.point_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    trajectory_control_node = TrajectoryControlNode()

    rclpy.spin(trajectory_control_node)

    trajectory_control_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()