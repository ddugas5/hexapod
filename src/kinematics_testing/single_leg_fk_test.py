import numpy as np


def forward_kinematics(hip_angle, femur_angle, tibia_angle):
    """
    Compute foot position in Cartesian space given joint angles (absolute).
    
    Args:
        hip_angle: rotation around X axis (rad)
        femur_angle: angle in XY plane (rad)
        tibia_angle: angle in XY plane (rad)
    
    Returns:
        foot_x, foot_y, foot_z: position of foot in world frame (inches)
        (X: forward, Y: left-right relative to body, Z: up-down)
    """
    
    femur_length = 4.272  # femur segment (inches)
    tibia_length = 6.217  # tibia segment (inches)
    
    # Femur extends from origin
    femur_x = femur_length * np.cos(femur_angle) * np.cos(hip_angle)
    femur_y = femur_length * np.cos(femur_angle) * np.sin(hip_angle)
    femur_z = -femur_length * np.sin(femur_angle)
    
    # Tibia extends from femur
    foot_x = femur_x + tibia_length * np.cos(tibia_angle) * np.cos(hip_angle)
    foot_y = femur_y + tibia_length * np.cos(tibia_angle) * np.sin(hip_angle)
    foot_z = femur_z - tibia_length * np.sin(tibia_angle)
    
    return foot_x, foot_y, foot_z


# Get angle inputs in degrees
hip_angle_deg = float(input("Hip angle (degrees): "))
femur_angle_deg = float(input("Femur angle (degrees): "))
tibia_angle_deg = float(input("Tibia angle (degrees): "))

# Convert to radians and calculate foot position
hip_angle = np.deg2rad(hip_angle_deg)
femur_angle = np.deg2rad(femur_angle_deg)
tibia_angle = np.deg2rad(tibia_angle_deg)

foot_x, foot_y, foot_z = forward_kinematics(hip_angle, femur_angle, tibia_angle)

# Output foot location
print(f"Foot position: ({foot_x:.3f}, {foot_y:.3f}, {foot_z:.3f}) inches")
