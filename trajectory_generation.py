import modern_robotics as mr
import numpy as np
import pandas as pd


def create_trajectories(trajectories, gripper_state, output_trajectories):
    if gripper_state:
        for traj in trajectories:
            output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 1])
            output_trajectories.append(output_trajectory)    
            
            if np.array_equal(traj, trajectories[0]):
                for _ in range(63): 
                    output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 1])
                    output_trajectories.append(output_trajectory)
    else:
        for traj in trajectories:
            output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 0])
            output_trajectories.append(output_trajectory)
        

def Trajectory_Generation(Tse_initial: np.ndarray, Tsc_initial: np.ndarray, Tsc_final: np.ndarray, Tce_grasp: np.ndarray, Tce_standoff: np.ndarray, k: int, gripper_state: bool):
    """
    Write output trajectories to a csv for Coppelia Sim to use at each timestep

    Args:
        T_se_initial (np.ndarray): _description_
        Tsc_initial (np.ndarray): _description_
        Tsc_final (np.ndarray): _description_
        Tce_grasp (np.ndarray): _description_
        Tce_standoff (np.ndarray): _description_
        k (int): _description_ 
    """
    output_trajectories_ee = []
    output_trajectories = []
    gripper_state_list = []
    
    #Get the e-e transform in the s frame
    Tse_standoff_initial = Tsc_initial @ Tce_standoff
    trajectories1 = mr.ScrewTrajectory(Xstart=Tse_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories1)
    gripper_state_list.extend(np.zeros(len(trajectories1)))
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)
    
    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    
    trajectories2 = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_grasp_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories2)
    gripper_state_list.extend(np.zeros(len(trajectories2)))
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)
    
    for _ in range(63):
        output_trajectories.append(output_trajectories[-1])
    gripper_state_list.extend(np.ones(63))
    
    trajectories3 = mr.ScrewTrajectory(Xstart=Tse_grasp_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories3)
    gripper_state_list.extend(np.ones(len(trajectories3)))
    
    
    #gripper_state = True
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)
    
    Tse_standoff_final = Tsc_final @ Tce_standoff
    trajectories4 = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories4)
    gripper_state_list.extend(np.ones(len(trajectories4)))
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)
    
    Tse_grasp_final = Tsc_final @ Tce_grasp
    trajectories5 = mr.ScrewTrajectory(Xstart=Tse_standoff_final, Xend=Tse_grasp_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories5)
    gripper_state_list.extend(np.ones(len(trajectories5)))
    
    for _ in range(63):
        output_trajectories.append(output_trajectories[-1])
    gripper_state_list.extend(np.zeros(63))
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)
    
    trajectories6 = mr.ScrewTrajectory(Xstart=Tse_grasp_final, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    output_trajectories.extend(trajectories6)
    gripper_state_list.extend(np.zeros(len(trajectories6)))
    #gripper_state = False
    #create_trajectories(trajectories, gripper_state, output_trajectories_ee)

    # output_trajectories_df = pd.DataFrame(output_trajectories_ee)
    # output_trajectories_df.to_csv("output_trajectories.csv", index=False, header=False)
    
    return output_trajectories, gripper_state_list
    
    