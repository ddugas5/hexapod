import numpy as np

def forward_kinematics(hip_angle, femur_angle, tibia_angle):
    """
    Compute foot position in Cartesian space given joint angles (absolute).
    Assumes femur_angle and tibia_angle are in the body frame with mounting offsets already removed.
    
    Args:
        hip_angle: rotation around Z axis (rad)
        femur_angle: angle from horizontal (rad) - already in body frame
        tibia_angle: angle from horizontal (rad) - already in body frame
    
    Returns:
        foot_x, foot_y, foot_z: position of foot relative to body frame (inches)
    """
    
    femur_length = 4.272  # femur segment (inches)
    tibia_length = 6.217  # tibia segment (inches)
    coxa_length = 2.743   # coxa segment (inches)
    
    # Femur extends from coxa joint (accounting for coxa length offset)
    femur_x = coxa_length + femur_length * np.cos(femur_angle) * np.cos(hip_angle)
    femur_y = femur_length * np.cos(femur_angle) * np.sin(hip_angle)
    femur_z = -femur_length * np.sin(femur_angle)
    
    # Tibia extends from femur
    foot_x = femur_x + tibia_length * np.cos(tibia_angle) * np.cos(hip_angle)
    foot_y = femur_y + tibia_length * np.cos(tibia_angle) * np.sin(hip_angle)
    foot_z = femur_z - tibia_length * np.sin(tibia_angle)
    
    return foot_x, foot_y, foot_z

def calibrate_z_offset():
    """
    Calculate the z_offset by putting the leg in a known position and measuring where it actually is.
    The z_offset accounts for the height of the femur joint mounting above the body frame.
    From URDF: coxa_z = 0.608 inches, femur_z = 0.544 inches = 1.152 inches total
    """
    print("=" * 60)
    print("Z-Offset Calibration for Inverse Kinematics")
    print("=" * 60)
    print("The z_offset is the cumulative Z-height of joint frames from the URDF")
    print("This allows us to transform target positions from body frame to femur joint frame")
    
    # From URDF (converting meters to inches):
    # middle_left_coxa_joint z = 0.0154258 m = 0.6075 inches
    # middle_left_femur_joint z = 0.0138198 m = 0.5441 inches
    urdf_z_offset = 0.6075 + 0.5441  # = 1.1516 inches
    
    print(f"\nFrom URDF analysis:")
    print(f"  Coxa joint z-offset: 0.6075 inches")
    print(f"  Femur joint z-offset: 0.5441 inches")
    print(f"  Total URDF z-offset: {urdf_z_offset:.4f} inches")
    
    # Now test with inverse kinematics logic
    # When we specify target (x, y, z) in body frame and add z_offset,
    # we're converting to the coordinate system where femur joint is origin
    
    print("\n" + "=" * 60)
    print("Verification: Testing with known positions")
    print("=" * 60)
    
    test_configs = [
        (0.0, 0.0, 0.0, "All zeros (resting)"),
        (0.0, np.deg2rad(30), 0.0, "Femur 30°"),
        (0.0, np.deg2rad(-30), 0.0, "Femur -30°"),
        (0.0, 0.0, np.deg2rad(30), "Tibia 30°"),
    ]
    
    print("\nIf z_offset is correct, when you specify target (0, 5, z_offset),")
    print("the foot should actually reach z=0 in body frame.")
    
    for hip, femur, tibia, desc in test_configs:
        x, y, z = forward_kinematics(hip, femur, tibia)
        print(f"\n{desc}:")
        print(f"  Foot position in body frame: ({x:.4f}, {y:.4f}, {z:.4f}) inches")
    
    print("\n" + "=" * 60)
    print(f"Recommended z_offset value: {urdf_z_offset:.4f} inches")
    print(f"Current code uses: 1.375 inches")
    print(f"Difference: {abs(1.375 - urdf_z_offset):.4f} inches")
    print("=" * 60)

if __name__ == '__main__':
    calibrate_z_offset()
