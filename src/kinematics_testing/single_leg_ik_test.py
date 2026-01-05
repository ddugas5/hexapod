import numpy as np 


    def solve_angles(x, y, z):

        fl =
        tl =
        l = np.sqrt(y**2 + z**2)

        ta = np.arccos((fl**2 + tl**2- l**2)/(2*fl*tl))
        fa = np.arccos((tl**2 + l**2 - fl**2)/2*tl*l)
        phi_f = np.arctan2(z,y)

        theta_f = phi_f - fa

        return ta, theta_f

#calulate in joint space       
x = 0.0
y = 4.0
z = -2.0

ga = 
ta, theta_f = solve_angles(z, y)

femur_angle = theta_f
tibia_angle = ta + ga
hip_angle = np.arctan2(y,x)

print(
    f"Calculated Joint Space: hip_angle = {hip_angle:.2f}, femur_angle = {femur_angle:.2f}, tibia_angle = {tibia_angle:.2f},
     hip_angle_deg = {hip_angle_deg:.2f}, femur_angle_deg = {femur_angle_deg:.2f}, tibia_angle_deg = {tibia_angle_deg:.2f}"
)

#convert to motor space


print(
    f"Calculated Motor Space: "
)