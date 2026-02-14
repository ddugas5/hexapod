import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
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

        self.subscription = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)

        #leg bases relative to body center
        self.nominal = [
            [ 2, 3.5],   # front left
            [ 0, 3.5],   # middle left
            [ -2, 3.5],   # rear left
            [ 2, 3.5],   # front right
            [ 0, 3.5],   # middle right
            [ -2, 3.5],   # rear right
        ]

        self.gait_period = 1.0 #sec
        self.start_time = time.time()

        self.step_height = 2.0 #in
        self.stride = 3.0

        self.vx = 0.0
        self.wz = 0.0

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

    def joystick_callback(self, msg):
        self.right_joystick = msg.axes[4]
        self.left_joystick = msg.axes[0]

        #scale for increasing speed
        self.vx = self.right_joystick * 3.0
        self.wz = self.left_joystick

        # self.get_logger().info(f"vx={self.vx:.2f}, wz={self.wz:.2f}")

    def timer_callback(self):
        if abs(self.vx) < 0.01 and abs(self.wz) < 0.01:
            #set all legs to standing position
            targets=[]
            for leg_idx in range(6):
                bx, by = self.nominal[leg_idx]
                rot_x = bx * np.cos(0) - by * np.sin(0)
                rot_y = bx * np.sin(0) + by * np.cos(0)
                foot_x = rot_x
                foot_y = rot_y
                foot_z = -2.0
                targets.extend([foot_x, foot_y, foot_z])

        else:

            t = (time.time() - self.start_time) / self.gait_period #make global timer and phase calculation
            phase = t % 1.0 #take the fractional part of t and you will always have a phase between 0 & 1
            body_x = self.vx * self.gait_period * phase #body movement in the x direction
            theta = self.wz * self.gait_period * phase #body angle rotation

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
                    step = self.vx * self.gait_period
                    x = -step / 2 + s * step #linear interpolation for the x linear movement of the swing
                    z = -2.0 + self.step_height * np.sin(np.pi * s) #z vertical lift in a sin wave from 0 -> pi

                else:
                    #stance phase
                    s = (leg_phase - 0.5) / 0.5 #again normalize 0 -> 1 for the stance phase time
                    step = self.vx * self.gait_period
                    # x = step / 2 - s * step #linear interpolate the x linear movement for the push along the floor
                    x = -s * step
                    z = -2.0 #z stays on the ground

                # y = 3.0 #per leg offsets later
                # targets.extend([x - body_x, y, z]) #append 3D position to the list
                bx, by = self.nominal[leg_idx]

                # rotate nominal point
                rot_x = bx * np.cos(theta) - by * np.sin(theta)
                rot_y = bx * np.sin(theta) + by * np.cos(theta)

                foot_x = rot_x + x - body_x
                foot_y = rot_y
                foot_z = z

                targets.extend([foot_x, foot_y, foot_z])


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