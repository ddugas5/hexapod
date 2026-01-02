# Hexapod

YouTube Demo: (coming soon)
  
## Overview
This is a small hexapod robot that I am building as a personal project to gain experience as a robotics engineer. It has 18 degrees of freedom and is made from PLA 3D printed parts. I designed it in Solidworks and wrote the code for it on a Raspberry Pi 4B in ROS2/Python. As for the rest of my electronics, I made a custom servo driver and this was powered using a 7.4V 8000 mAh LiPo battery. I stepped this down to 5V using a DEVMO DC-DC Boot Buck Converter to power the Pi as well as the servos.

## Code
There are several ROS Nodes set up in this workspace. These are the core of what makes the hexapod move and how they all work together.
* servo_control_node
  * Subscribes to "/joint_angles" and then outputs a pwm to servo motors to put them to that angle
  * Also subscribes to "/servo/joint_5/cmd" to set the gripper to open or closed
* 

## Math  
