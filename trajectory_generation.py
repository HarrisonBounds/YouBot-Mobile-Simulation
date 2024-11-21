import modern_robotics as mr
import numpy as np
import pandas as pd
import os

Tse_initial = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.0963+0.0026+0.6546],
                        [0, 0, 0, 0]])

Tsc_initial = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

Tsc_final = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

Tce_grasp = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.025],
                      [0, 0, 0, 1]])

Tce_standoff = np.array([[0, 0, 1, 0],
                         [0, 1, 0, 0],
                         [-1, 0, 0, 0.5],
                         [0, 0, 0, 1]])

k = 1

output_file = "output_trajectories.csv"
output_trajectories = []

def create_trajectories(trajectories, file_path):
    #Append trajectories if the path already exists
    if os.path.exists(file_path):
        for traj in trajectories:
            output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 0])
            output_trajectories.append(output_trajectory)
    else:
        for traj in trajectories:
            output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 0])
            output_trajectories.append(output_trajectory)
            
def create_frame_list(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff):
    Tse_standoff_initial = Tsc_initial @ Tce_standoff
    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    Tse_standoff_final = Tsc_final @ Tce_standoff
    Tse_grasp_final = Tsc_final @ Tce_grasp
    frame_list = []

def Trajectory_Generation(Tse_initial: np.ndarray, Tsc_initial: np.ndarray, Tsc_final: np.ndarray, Tce_grasp: np.ndarray, Tce_standoff: np.ndarray, k: int, output_file: str):
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
    
    #Get the e-e transform in the s frame
    Tse_standoff_initial = Tsc_initial @ Tce_standoff
    trajectories = mr.ScrewTrajectory(Xstart=Tse_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)
    
    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    
    trajectories = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_grasp_initial, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)
    
    trajectories = mr.ScrewTrajectory(Xstart=Tse_grasp_initial, Xend=Tse_standoff_initial, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)
    
    Tse_standoff_final = Tsc_final @ Tce_standoff
    trajectories = mr.ScrewTrajectory(Xstart=Tse_standoff_initial, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)
    
    Tse_grasp_final = Tsc_final @ Tce_grasp
    trajectories = mr.ScrewTrajectory(Xstart=Tse_standoff_final, Xend=Tse_grasp_final, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)
    
    trajectories = mr.ScrewTrajectory(Xstart=Tse_grasp_final, Xend=Tse_standoff_final, Tf=3, N=3*k/0.01, method=3)
    create_trajectories(trajectories, output_file)

    
    output_trajectories_df = pd.DataFrame(output_trajectories)
    output_trajectories_df.to_csv("output_trajectories.csv", index=False, header=False)
    
Trajectory_Generation(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, output_file)
    