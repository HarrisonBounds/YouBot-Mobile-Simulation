import modern_robotics as mr
import numpy as np

def Trajectory_Generation(Tse_initial: np.ndarray, Tsc_initial: np.ndarray, Tsc_final: np.ndarray, Tce_grasp: np.ndarray, Tce_standoff: np.ndarray, k: int, gripper_state: bool):
    """
    Generates a sequence of end-effector trajectories for a robotic arm to execute a pick-and-place task.

    This function computes the trajectory of the end-effector (EE) through six key phases:
    1. Moving to the initial standoff position above the cube.
    2. Lowering to the grasp position.
    3. Holding the grasp position.
    4. Returning to the standoff position.
    5. Moving to the final standoff position above the placement location.
    6. Lowering to place the object, holding, and returning.

    Args:
        Tse_initial (np.ndarray): Initial transformation matrix of the end-effector in the space frame.
        Tsc_initial (np.ndarray): Transformation matrix of the initial cube position in the space frame.
        Tsc_final (np.ndarray): Transformation matrix of the final cube position in the space frame.
        Tce_grasp (np.ndarray): Transformation matrix of the end-effector relative to the cube when grasping.
        Tce_standoff (np.ndarray): Transformation matrix of the end-effector relative to the cube in a standoff position.
        k (int): Gain factor to control trajectory resolution (affects time steps and granularity).
        gripper_state (bool): Initial state of the gripper (open or closed).

    Returns:
        tuple: A tuple containing:
            - output_trajectories (list): A list of transformation matrices representing the end-effector's trajectory.
            - gripper_state_list (list): A list of gripper states corresponding to each trajectory step.
    """
    output_trajectories_ee = []  # List to store end-effector trajectories
    output_trajectories = []    # Complete trajectory output
    gripper_state_list = []     # Corresponding gripper states for each trajectory step

    # Phase 1: Move to the standoff position above the cube's initial location
    Tse_standoff_initial = Tsc_initial @ Tce_standoff
    trajectories1 = mr.ScrewTrajectory(Xstart=Tse_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories1)
    gripper_state_list.extend(np.zeros(len(trajectories1)))  # Gripper remains open

    # Phase 2: Lower the end-effector to the grasp position
    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    trajectories2 = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_grasp_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories2)
    gripper_state_list.extend(np.zeros(len(trajectories2)))  # Gripper remains open

    # Phase 3: Hold the grasp position
    for _ in range(63):  # Hold for a fixed duration
        output_trajectories.append(output_trajectories[-1])  # Repeat last trajectory step
    gripper_state_list.extend(np.ones(63))  # Gripper closes

    # Phase 4: Return to the standoff position
    trajectories3 = mr.ScrewTrajectory(Xstart=Tse_grasp_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories3)
    gripper_state_list.extend(np.ones(len(trajectories3)))  # Gripper remains closed

    # Phase 5: Move to the standoff position above the placement location
    Tse_standoff_final = Tsc_final @ Tce_standoff
    trajectories4 = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories4)
    gripper_state_list.extend(np.ones(len(trajectories4)))  # Gripper remains closed

    # Phase 6: Lower to the placement position and release the object
    Tse_grasp_final = Tsc_final @ Tce_grasp
    trajectories5 = mr.ScrewTrajectory(Xstart=Tse_standoff_final, Xend=Tse_grasp_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories5)
    gripper_state_list.extend(np.ones(len(trajectories5)))  # Gripper remains closed

    for _ in range(63):  # Hold for a fixed duration
        output_trajectories.append(output_trajectories[-1])  # Repeat last trajectory step
    gripper_state_list.extend(np.zeros(63))  # Gripper opens

    # Return to the standoff position after releasing the object
    trajectories6 = mr.ScrewTrajectory(Xstart=Tse_grasp_final, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories6)
    gripper_state_list.extend(np.zeros(len(trajectories6)))  # Gripper remains open

    return output_trajectories, gripper_state_list
