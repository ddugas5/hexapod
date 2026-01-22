
import rclpy
import time
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/foot_goal',
            self.kinematics_callback,
            10
        )

        #publish joint states to be used by the urdf
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        #publish to servo controller (use in the future)
        self.ik_publisher = self.create_publisher(Float32MultiArray, '/joint_angles', 10)

        #publish degrees for debugging
        self.deg_publisher = self.create_publisher(Float32MultiArray, '/joint_angles/degrees', 10)

        #define leg configuration
        self.legs = {
            "front_left": {
                "coxa": "front_left_coxa_joint",
                "femur": "front_left_femur_joint",
                "tibia": "front_left_tibia_joint",
                "signs": [-1, -1, 1]
            },
            "front_right": {
                "coxa": "front_right_coxa_joint",
                "femur": "front_right_femur_joint",
                "tibia": "front_right_tibia_joint",
                "signs": [-1, 1, 1]
            },
            "middle_left": {
                "coxa": "middle_left_coxa_joint",
                "femur": "middle_left_femur_joint",
                "tibia": "middle_left_tibia_joint",
                "signs": [-1, -1, 1]
            },
            "middle_right": {
                "coxa": "middle_right_coxa_joint",
                "femur": "middle_right_femur_joint",
                "tibia": "middle_right_tibia_joint",
                "signs": [-1, -1, 1]
            },
            "rear_left": {
                "coxa": "rear_left_coxa_joint",
                "femur": "rear_left_femur_joint",
                "tibia": "rear_left_tibia_joint",
                "signs": [1, 1, 1]
            },
            "rear_right": {
                "coxa": "rear_right_coxa_joint",
                "femur": "rear_right_femur_joint",
                "tibia": "rear_right_tibia_joint",
                "signs": [1, -1, 1]
            },
        }


        #make dict of all joint angles
        self.joint_angles = {
            'front_left_coxa_joint': 0.0,
            'front_left_femur_joint': 0.0,
            'front_left_tibia_joint': 0.0,
            'middle_left_coxa_joint': 0.0,
            'middle_left_femur_joint': 0.0,
            'middle_left_tibia_joint': 0.0,
            'rear_left_femur_joint': 0.0,
            'rear_left_tibia_joint': 0.0,
            'front_right_coxa_joint': 0.0,
            'front_right_femur_joint': 0.0,
            'front_right_tibia_joint': 0.0,
            'middle_right_coxa_joint': 0.0,
            'middle_right_femur_joint': 0.0,
            'middle_right_tibia_joint': 0.0,
            'rear_right_coxa_joint': 0.0,
            'rear_right_femur_joint': 0.0,
            'rear_right_tibia_joint': 0.0,
            'rear_left_coxa_joint': 0.0,
        }

        self.joint_angles_deg = {
            'front_left_coxa_joint': 0.0,
            'front_left_femur_joint': 0.0,
            'front_left_tibia_joint': 0.0,
            'middle_left_coxa_joint': 0.0,
            'middle_left_femur_joint': 0.0,
            'middle_left_tibia_joint': 0.0,
            'rear_left_femur_joint': 0.0,
            'rear_left_tibia_joint': 0.0,
            'front_right_coxa_joint': 0.0,
            'front_right_femur_joint': 0.0,
            'front_right_tibia_joint': 0.0,
            'middle_right_coxa_joint': 0.0,
            'middle_right_femur_joint': 0.0,
            'middle_right_tibia_joint': 0.0,
            'rear_right_coxa_joint': 0.0,
            'rear_right_femur_joint': 0.0,
            'rear_right_tibia_joint': 0.0,
            'rear_left_coxa_joint': 0.0,
        }

    #function to calculate the tibia angle and femur angle
    def solve_angles(self, z, y):
        fl = 4.272 #femur length
        tl = 6.217 #tibia length
        l = np.sqrt(y**2 + z**2)  

        ta = np.arccos((fl**2 + tl**2 - l**2)/(2*fl*tl))
        fa = np.arccos((tl**2 + l**2 - fl**2)/(2*tl*l))
        phi_f = np.arctan2(z,y)

        theta_f = phi_f + fa #final femur_angle in rad

        return ta, theta_f

    def solve_leg_ik(self, x, y, z):

        # #extract the point
        # x = msg.x
        # y = msg.y
        # z = msg.z

        #coxa length
        coxa_length = 2.743 #inches
        z_offset = 3.0

        #horizontal distance
        h = np.sqrt(x**2 + y**2)

        #distance from femur joint
        r = h + coxa_length

        #adjust z to get to femur frame from body frame
        z_adjusted = z + z_offset

        ta, theta_f = self.solve_angles(z_adjusted, r)

        femur_angle = theta_f #final femur angle in rad
        tibia_angle = ta  #final tibia angle in rad
        hip_angle = np.arctan2(y,x) #final hip angle in rad
        final_hip_angle = hip_angle - 1.570796

        return final_hip_angle, femur_angle, tibia_angle

    def kinematics_callback(self, msg):

        data = msg.data

        foot_targets = {
            "front_left":   data[0:3],
            "front_right":  data[3:6],
            "middle_left":  data[6:9],
            "middle_right": data[9:12],
            "rear_left":    data[12:15],
            "rear_right":   data[15:18],
        }

        for leg_name, cfg in self.legs.items():
            x, y, z = foot_targets[leg_name]

            angles = self.solve_leg_ik(x, y, z)

            for joint, angle, sign in zip(
                [cfg["coxa"], cfg["femur"], cfg["tibia"]],
                angles,
                cfg["signs"]
            ):
                self.joint_angles[joint] = sign * angle
                self.joint_angles_deg[joint] = np.rad2deg(angle)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(self.joint_angles.keys())
        joint_msg.position = list(self.joint_angles.values())

        self.joint_state_publisher.publish(joint_msg)

        #publish to servo node
        angle_msg = Float32MultiArray()
        angle_msg.data = list(self.joint_angles.values())
        self.ik_publisher.publish(angle_msg)

        deg_angle_msg = Float32MultiArray()
        deg_angle_msg.data = list(self.joint_angles_deg.values())
        self.deg_publisher.publish(deg_angle_msg)


def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_node = InverseKinematicsNode()

    rclpy.spin(inverse_kinematics_node)

    inverse_kinematics_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()