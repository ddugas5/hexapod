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

        self.gait_period = 1.0 #sec
        self.start_time = time.time()

        self.step_height = 2.0 #in
        self.stride = 2.5

        #leg index mapping
        self.tripod_A = [0, 2, 4] #front left, rear left, middle right
        self.tripod_B = [1, 3, 5] #middle left, front right, rear right

        #trajectory parameters
        # self.step_height = 2.0 #in
        self.duration = 1.0 #sec
        self.rate = 100.0 #hz

        #start and end foot positions for both stand and swing
        self.swing_start = np.array([-2.0, 3.0, -2.0])
        self.swing_end = np.array([2.0, 3.0, -2.0])

        self.stance_start = np.array([2.0, 3.0, -2.0])
        self.stance_end = np.array([-2.0, 3.0, -2.0])

        #set initial phase and time
        # self.phase = "swing"
        # self.start_time = time.time()

        self.timer = self.create_timer(
            1.0/self.rate, self.timer_callback
        )

    def timer_callback(self):
        # t = (time.time() - self.start_time) / self.duration
        # t = np.clip(t, 0.0, 1.0)

        # if self.phase == "swing":
        #     #loop through arrays of "start" and "end" and linear interpolate x, y, and z linearly
        #     pos = self.swing_start + t * (self.swing_end - self.swing_start)

        #     #modify the 3rd value in the array to make the z go linear but have a sinusoidal lift
        #     pos[2] += self.step_height * np.sin(np.pi * t)

        # elif self.phase == "stance":
        #     pos = self.stance_start + t * (self.stance_end - self.stance_start)
        #     #no vertical lift here, this is where the leg moves back to original stance

        #publish array as a point
        # msg = Float32MultiArray()
        # x = pos[0]
        # y = pos[1]
        # z = pos[2]

        # msg.data = [
        #     0.0, 3.0, -2.0,
        #     0.0, 3.0, -2.0,
        #     0.0, 3.0, -2.0,
        #     0.0, 3.0, -2.0,
        #     0.0, 3.0, -2.0,
        #     0.0, 3.0, -2.0,
        # ]

        # msg.data[3] = x
        # msg.data[4] = y
        # msg.data[5] = z

        # self.goal_publisher.publish(msg)

        # #start timer over to make it repeat on loop
        # if t >= 1.0:
        #     self.start_time= time.time()

        #     if self.phase == "swing":
        #         self.phase = "stance"
        #     else:
        #         self.phase = "swing"

        t = (time.time() - self.start_time) / self.gait_period #make global timer and phase calculation
        phase = t % 1.0 #take the fractional part of t and you will always have a phase between 0 & 1

        targets = []

        #loop through all six legs in the index
        for leg_idx in range(6):
            
            leg_phase = phase #set the leg phase to what was calculated above
            #for legs in tripod B, add 0.5 to shift the phase by half (halfway through the phase is where you want 3/6 legs to be)
            if leg_idx in self.tripod_B:
                leg_phase = (phase + 0.5) % 1.0 

            #condition is first half of the leg cycle
            if leg_phase < 0.5:
                #swing phase
                s = leg_phase / 0.5 #normalize it to 0->1 for only the first half of the phase (s=0 is start of swing, s=1 is end of swing)
                x = -self.stride / 2 + s * self.stride #linear interpolation for the x linear movement of the swing
                z = -2.0 + self.step_height * np.sin(np.pi * s) #z vertical lift in a sin wave from 0 -> pi

            else:
                #stance phase
                s = (leg_phase - 0.5) / 0.5 #again normalize 0 -> 1 for the stance phase time
                x = self.stride / 2 - s * self.stride #linear interpolate the x linear movement for the push along the floor
                z = -2.0 #z stays on the ground

            y = 3.0 #per leg offsets later
            targets.extend([x, y, z]) #append 3D position to the list

        msg = Float32MultiArray()
        msg.data = targets
        self.goal_publisher.publish(msg)

def main():
    rclpy.init()

    foot_trajectory_controller = FootTrajectoryController()
    
    rclpy.spin(foot_trajectory_controller)
    
    foot_trajectory_controller.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()