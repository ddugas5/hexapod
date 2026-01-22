import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_servokit import ServoKit

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class MultiServoControlNode(Node):
    def __init__(self):
        super().__init__('multi_servo_control_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self.angle_callback,
            10
        )

        self.kit = ServoKit(channels=16)
    
        #store all channels, ranges, and whether to invert the servo or not
        self.servos = [
            {"channel": 0, "servo_min": 50, "servo_max": 120, "min_rad": 0.872665, "max_rad": 2.0944, "invert": False},
            {"channel": 1, "servo_min": 50, "servo_max": 180, "min_rad": 0.872665, "max_rad": 3.1415, "invert": True},
            {"channel": 2, "servo_min": 0, "servo_max": 135, "min_rad": 0.0, "max_rad": 2.3561, "invert": False},
            {"channel": 3, "servo_min": 50, "servo_max": 120, "min_rad": -1.5708, "max_rad": 1.5708, "invert": False},  # middle_left_coxa
            {"channel": 4, "servo_min": 50, "servo_max": 180, "min_rad": -1.5708, "max_rad": 1.5708, "invert": True},   # middle_left_femur
            {"channel": 5, "servo_min": 0, "servo_max": 135, "min_rad": 0.0, "max_rad": 3.1415, "invert": True},        # middle_left_tibia
        ]

    def angle_callback(self, msg):
        # Only process middle left leg (indices 3, 4, 5)
        for i in range(3, 6):
            if i >= len(msg.data):
                break
            
            joint_angle = msg.data[i]
            configuration = self.servos[i]
        
        # # Original version - process all servos
        # for i, joint_angle in enumerate(msg.data):
        #     configuration = self.servos[i]

            #clamp radians
            joint_angle = max(configuration["min_rad"], min(configuration["max_rad"], joint_angle))

            #normalize to [0, 1]
            norm = (joint_angle - configuration["min_rad"]) / (
                configuration["max_rad"] - configuration["min_rad"]
            )
            
            #if the servo is inverted, change the sign
            if configuration["invert"]:
                norm = 1.0 - norm

            #convert to degrees (servo space)
            servo_angle = configuration["servo_min"] + norm * (configuration["servo_max"]
                        - configuration["servo_min"])
            
            #send to servo
            self.kit.servo[configuration["channel"]].angle = servo_angle

def main(args=None):
    rclpy.init(args=args)

    multi_servo_control_node = MultiServoControlNode()

    rclpy.spin(multi_servo_control_node)

    multi_servo_control_node.destoy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
