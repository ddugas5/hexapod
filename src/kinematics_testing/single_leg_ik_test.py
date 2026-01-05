import numpy as np 


def solve_angles(z, y):

    fl = 4.272 #femur length
    tl = 6.217 #tibia length
    l = np.sqrt(y**2 + z**2)  

    ta = np.arccos((fl**2 + tl**2 - l**2)/(2*fl*tl))
    fa = np.arccos((tl**2 + l**2 - fl**2)/(2*tl*l))
    phi_f = np.arctan2(z,y)

    theta_f = phi_f + fa #final femur_angle in rad

    return ta, theta_f

#calulate in joint space       
x = 0.0
y = 0.0
z = 0.0


ta, theta_f = solve_angles(z, y)

femur_angle = theta_f #final femur angle in rad
tibia_angle = ta  #final tibia angle in rad
hip_angle = np.arctan2(y,x) #final hip angle in rad

#change to degrees
femur_angle_deg = np.rad2deg(femur_angle)  #final femur angle in deg
tibia_angle_deg = np.rad2deg(tibia_angle) #final tibia angle in deg
hip_angle_deg = np.rad2deg(hip_angle) #final hip angle in deg

print(
    f"Calculated Joint Space: hip_angle = {hip_angle:.2f}, femur_angle = {femur_angle:.2f}, tibia_angle = {tibia_angle:.2f}, hip_angle_deg = {hip_angle_deg:.2f}, femur_angle_deg = {femur_angle_deg:.2f}, tibia_angle_deg = {tibia_angle_deg:.2f}"
)

#convert to motor space

# ms_femur_angle = femur_angle
# ms_tibia_angle = tibia_angle
# ms_hip_angle = hip_angle

# ms_femur_angle_deg = np.rad2deg(ms_femur_angle)
# ms_tibia_angle_deg = np.rad2deg(ms_tibia_angle)
# ms_hip_angle_deg = np.rad2deg(ms_hip_angle)

# print(
#     f"Calculated Motor Space: hip_angle = {ms_hip_angle:.2f}, femur_angle = {ms_femur_angle:.2f}, tibia_angle = {ms_tibia_angle:.2f}, hip_angle_deg = {ms_hip_angle_deg:.2f}, femur_angle_deg = {ms_femur_angle_deg:.2f}, tibia_angle_deg = {ms_tibia_angle_deg:.2f}"
# )