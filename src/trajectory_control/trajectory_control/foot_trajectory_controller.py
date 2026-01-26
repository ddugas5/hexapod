import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class FootTrajectoryController(Node):
    def __init__(self):
        super().__init__('foot_trajectory_controller')

        self.goal_publisher = self.create_publisher(
            Float32MultiArray,
            '/foot_goal',
            10
        )

        #trajectory parameters
        self.step_height = 2.0 #in
        self.duration = 1.0 #sec
        self.rate = 100.0 #hz

        #start and end foot positions for both stand and swing
        self.swing_start = np.array([-2.0, 3.0, -2.0])
        self.swing_end = np.array([2.0, 3.0, -2.0])

        self.stance_start = np.array([2.0, 3.0, -2.0])
        self.stance_end = np.array([-2.0, 3.0, -2.0])

        #set initial phase and time
        self.phase = "swing"
        self.start_time = time.time()

        self.timer = self.create_timer(
            1.0/self.rate, self.timer_callback
        )

    def timer_callback(self):
        t = (time.time() - self.start_time) / self.duration
        t = np.clip(t, 0.0, 1.0)

        if self.phase == "swing":
            #loop through arrays of "start" and "end" and linear interpolate x, y, and z linearly
            pos = self.swing_start + t * (self.swing_end - self.swing_start)

            #modify the 3rd value in the array to make the z go linear but have a sinusoidal lift
            pos[2] += self.step_height * np.sin(np.pi * t)

        elif self.phase == "stance":
            pos = self.stance_start + t * (self.stance_end - self.stance_start)
            #no vertical lift here, this is where the leg moves back to original stance

        #publish array as a point
        msg = Float32MultiArray()
        x = pos[0]
        y = pos[1]
        z = pos[2]

        msg.data = [
            0.0, 3.0, -2.0,
            0.0, 3.0, -2.0,
            0.0, 3.0, -2.0,
            0.0, 3.0, -2.0,
            0.0, 3.0, -2.0,
            0.0, 3.0, -2.0,
        ]

        msg.data[3] = x
        msg.data[4] = y
        msg.data[5] = z

        self.goal_publisher.publish(msg)

        #start timer over to make it repeat on loop
        if t >= 1.0:
            self.start_time= time.time()

            if self.phase == "swing":
                self.phase = "stance"
            else:
                self.phase = "swing"

def main():
    rclpy.init()

    foot_trajectory_controller = FootTrajectoryController()
    
    rclpy.spin(foot_trajectory_controller)
    
    foot_trajectory_controller.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()