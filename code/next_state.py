import modern_robotics as mr
import numpy as np

def Next_State(current_config, controls, dt, angular_range):
    """
    Computes the next state of a mobile manipulator robot given the current configuration,
    control inputs, and time step.

    The function updates the robot's configuration, including the arm joint angles, wheel
    angles, and chassis pose, based on the control inputs and kinematic constraints.

    Args:
        current_config (np.ndarray): Current configuration of the robot as a 13-element vector:
            - [phi, x, y] (chassis orientation and position)
            - [theta_1, ..., theta_5] (arm joint angles)
            - [wheel_1, ..., wheel_4] (wheel angles)
        controls (np.ndarray): Control inputs as a 9-element vector:
            - [wheel_1_velocity, ..., wheel_4_velocity] (wheel angular velocities)
            - [theta_1_velocity, ..., theta_5_velocity] (arm joint angular velocities)
        dt (float): Time step for integration.
        angular_range (float): Maximum allowed angular velocity for the wheels and joints.

    Returns:
        np.ndarray: Updated 13-element configuration vector of the robot.
    """
    # Constants for the robot's physical dimensions
    l = 0.235  # Half of the distance between the front and rear wheels
    w = 0.15   # Half of the distance between the left and right wheels
    r = 0.0475 # Wheel radius

    # Initialize the new configuration with zeros
    new_config = np.zeros(13)

    # Clip the control inputs to the specified angular range
    controls = np.clip(controls, -angular_range, angular_range)

    # Update arm joint angles based on the control inputs
    for i in range(5):  # Loop through each arm joint
        new_config[i + 3] = current_config[i + 3] + controls[i + 4] * dt

    # Update wheel angles based on the control inputs
    for i in range(4):  # Loop through each wheel
        new_config[i + 8] = current_config[i + 8] + controls[i] * dt

    # Extract chassis pose variables
    phi = current_config[0]  # Chassis orientation
    x = current_config[1]    # Chassis x-position
    y = current_config[2]    # Chassis y-position

    # Chassis kinematic model matrix
    m = np.array([
        [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],  # Rotational velocity
        [1, 1, 1, 1],                                           # x-direction velocity
        [-1, 1, -1, 1]                                          # y-direction velocity
    ])

    # Compute chassis twist in the body frame
    V_b = (r / 4) * m @ np.array([
        new_config[8] - current_config[8],
        new_config[9] - current_config[9],
        new_config[10] - current_config[10],
        new_config[11] - current_config[11]
    ])

    # Convert twist to a 6D spatial vector (se3 representation)
    V_b6 = np.array([0, 0, V_b[0], V_b[1], V_b[2], 0])  # [wx, wy, wz, vx, vy, vz]
    V_b6 = mr.VecTose3(V_b6)  # Convert to a 4x4 transformation matrix

    # Compute the current transformation matrix (Tsb) of the chassis in the space frame
    Tsb = np.array([
        [np.cos(phi), -np.sin(phi), 0, x],
        [np.sin(phi), np.cos(phi), 0, y],
        [0, 0, 1, 0.0963],  # z-offset of the chassis
        [0, 0, 0, 1]
    ])

    # Compute the new transformation matrix after applying the twist
    Tsbk1 = Tsb @ mr.MatrixExp6(V_b6)

    # Update the chassis pose in the new configuration
    new_config[0] = np.arccos(Tsbk1[0][0])  # Updated orientation
    new_config[1] = Tsbk1[0][3]            # Updated x-position
    new_config[2] = Tsbk1[1][3]            # Updated y-position

    return new_config
