import modern_robotics as mr
import numpy as np
from feedforward_control import FeedForwardControl
from trajectory_generation import Trajectory_Generation
from next_state import Next_State
import pandas as pd
from plot import plot

# Initial Transformation Matrices
Tse_initial = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])

Tsc_initial = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 1],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

Tsc_final = np.array([[0, 1, 0, -1],
                      [-1, 0, 0, 0],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

Tce_grasp = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0],
                      [0, 0, 0, 1]])

Tce_standoff = np.array([[0, 0, 1, 0],
                         [0, 1, 0, 0],
                         [-1, 0, 0, 0.1],
                         [0, 0, 0, 1]])

T_angle = np.array([[np.cos(np.pi/3), 0, np.sin(np.pi/3), 0],
                    [0, 1, 0, 0],
                    [-np.sin(np.pi/3), 0, np.cos(np.pi/3), 0],
                    [0, 0, 0, 1]])

# Robot configuration for each base and joint
B1 = np.array([0, 0, 1, 0, 0.033, 0])
B2 = np.array([0, -1, 0, -0.5076, 0, 0])
B3 = np.array([0, -1, 0, -0.3526, 0, 0])
B4 = np.array([0, -1, 0, -0.2176, 0, 0])
B5 = np.array([0, 0, 1, 0, 0, 0])

B_list = np.array([B1, B2, B3, B4, B5]).T

Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])

M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])

# Constants related to the robot kinematics
l = 0.235
w = 0.15
r = 0.0475

# The Jacobian multiplier matrix
f = np.array([[0, 0, 0, 0],
                [0, 0, 0, 0],
                [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                [1, 1, 1 ,1],
                [-1, 1, -1, 1],
                [0, 0, 0, 0]])

f6 = f * 1/4 * r


def get_SE3(trajs: list):
    """
    Computes the current end-effector pose (Tse) based on the current configuration.

    Args:
        trajs (list): Current robot state containing joint angles and positions.

    Returns:
        Tse (np.ndarray): The end-effector transformation matrix (4x4).
    """
    phi = trajs[0]
    x = trajs[1]
    y = trajs[2]
    arm_config = trajs[3:8]
    
    # Transformation matrix for the base frame
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    
    # Transformation matrix for the body frame
    Tse = Tsb @ Tb0 @ mr.FKinBody(M0e, B_list, arm_config)
    
    return Tse


# Control gains
Kp = np.eye(6)
Ki = np.zeros((6, 6))

# Initial configuration of the robot (x, y, z, arm angles, etc.)
current_config = np.array([0, -0.382, 0, 0, 0.544, -0.831, -0.1291, 0, 0, 0, 0, 0, 0])
current_configs = []
x_errs = []
output_trajectories = []

# Generate output trajectories using the trajectory generation function
print("Creating output trajectories...")
output_trajectories, gripper_state_list = Trajectory_Generation(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp @ T_angle, Tce_standoff @ T_angle, k=1, gripper_state=False)

# Simulate robot motion and record joint configurations at each time step
print("Generating animation csv file...")
for i in range(len(output_trajectories) - 1):
    if i == 0:
        Tse = Tse_initial  # Initial pose
    else:
        Tse = get_SE3(current_config)  # Get current pose from configuration
    
    Tse_d = output_trajectories[i]
    Tse_d_next = output_trajectories[i+1]
    
    arm_theta_array = current_config[3:8]  # Extract arm joint angles
    T0e = mr.FKinBody(M0e, B_list, arm_theta_array)  # Forward kinematics to get end-effector pose
    J_arm = mr.JacobianBody(B_list, arm_theta_array)  # Arm Jacobian
    J_base = mr.Adjoint(mr.TransInv(T0e) @ mr.TransInv(Tb0)) @ f6  # Base Jacobian
    
    Je = np.hstack((J_base, J_arm))  # Full Jacobian (base + arm)
    
    # Compute joint velocities (control signals)
    wheel_and_arm_joint_speeds, x_err = FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, Je, dt=0.01)
    
    # Update current configuration based on the joint speeds
    current_config = Next_State(current_config, wheel_and_arm_joint_speeds, dt=0.01, angular_range=50)
    current_config[-1] = gripper_state_list[i]  # Update gripper state
    current_configs.append(current_config.copy())  # Record current config
    x_errs.append(x_err)  # Record end-effector error

# Save error and configuration data to CSV files
x_errs_df = pd.DataFrame(x_errs)
x_errs_df.to_csv("new_task_errors.csv", index=False, header=False) 
current_config_df = pd.DataFrame(current_configs)
current_config_df.to_csv("new_task_final.csv", index=False, header=False)

# Plot errors over time
print("Plotting errors...")
plot(x_errs)
print("Done!")
