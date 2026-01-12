
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.subscription = self.create_subscription(
            Point,
            '/foot_goal',
            self.kinematics_callback,
            10
        )

        #publish joint states to be used by the urdf
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_state', 10)

        #publish to servo controller (use in the future)
        self.ik_publisher = self.create_publisher(Float32MultiArray, '/joint_angles', 10)

        #make dict of all joint angles
        self.joint_angles = {
            'front_left_coxa_joint': 0.0,
            'front_left_femur_joint': 0.0,
            'front_left_tibia_joint': 0.0,
            'front_right_coxa_joint': 0.0,
            'front_right_femur_joint': 0.0,
            'front_right_tibia_joint': 0.0,
            'middle_left_coxa_joint': 0.0,
            'middle_left_femur_joint': 0.0,
            'middle_left_tibia_joint': 0.0,
            'middle_right_coxa_joint': 0.0,
            'middle_right_femur_joint': 0.0,
            'middle_right_tibia_joint': 0.0,
            'rear_left_coxa_joint': 0.0,
            'rear_left_femur_joint': 0.0,
            'rear_left_tibia_joint': 0.0,
            'rear_right_coxa_joint': 0.0,
            'rear_right_femur_joint': 0.0,
            'rear_right_tibia_joint': 0.0,
        }


    def kinematics_callback(self, msg):
        
        #function to calculate the tibia angle and femur angle
        def solve_angles(z, y):
            fl = 4.272 #femur length
            tl = 6.217 #tibia length
            l = np.sqrt(y**2 + z**2)  

            ta = np.arccos((fl**2 + tl**2 - l**2)/(2*fl*tl))
            fa = np.arccos((tl**2 + l**2 - fl**2)/(2*tl*l))
            phi_f = np.arctan2(z,y)

            theta_f = phi_f + fa #final femur_angle in rad

            return ta, theta_f 

        #extract the point
        x = msg.x
        y = msg.y
        z = msg.z

        ta, theta_f = solve_angles(z, y)

        femur_angle = theta_f #final femur angle in rad
        tibia_angle = ta  #final tibia angle in rad
        hip_angle = np.arctan2(y,x) #final hip angle in rad

        #change to degrees
        femur_angle_deg = np.rad2deg(femur_angle)  #final femur angle in deg
        tibia_angle_deg = np.rad2deg(tibia_angle) #final tibia angle in deg
        hip_angle_deg = np.rad2deg(hip_angle) #final hip angle in deg

        #update the middle left leg with math
        self.joint_angles['middle_left_coxa_joint'] = hip_angle
        self.joint_angles['middle_left_femur_joint'] = femur_angle
        self.joint_angles['middle_left_tibia_joint'] = tibia_angle

        #break dictionary into two lists
        joint_names = list(self.joint_angles.keys())
        joint_positions = list(self.joint_angles.values())

        #create joint state message and publish
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now()
        joint_msg.header.frame_id = "base_link"
        joint_msg.name = joint_names
        joint_msg.position = joint_positions

        self.joint_state_publisher.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_node = InverseKinematicsNode()

    rclpy.spin(inverse_kinematics_node)

    inverse_kinematics_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()