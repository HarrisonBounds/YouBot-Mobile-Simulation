import modern_robotics as mr
import numpy as np
import pandas as pd

Tse_initial = np.array([[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0]])

Tsc_initial = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

Tsc_final = np.array([[0, 1, 0, 1],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

Tce_grasp = np.array([0, 0, 0])
Tce_standoff = np.array([0, 0, 0])
k = 1

output_trajectories = []



def Trajectory_Generation(T_se_initial: np.ndarray, Tsc_initial: np.ndarray, Tsc_final: np.ndarray, Tce_grasp: np.ndarray, Tce_standoff: np.ndarray, k: int):
    """Write output trajectories to a csv for Coppelia Sim to use at each timestep

    Args:
        T_se_initial (np.ndarray): _description_
        Tsc_initial (np.ndarray): _description_
        Tsc_final (np.ndarray): _description_
        Tce_grasp (np.ndarray): _description_
        Tce_standoff (np.ndarray): _description_
        k (int): _description_ 
    """
    
    trajectories = mr.ScrewTrajectory(Xstart=T_se_initial, Xend=T_se_initial, Tf=20, N=8, method=3)
    
    for traj in trajectories:
        output_trajectory = np.array([traj[0][0], traj[0][1], traj[0][2], traj[1][0], traj[1][1], traj[1][2], traj[2][0], traj[2][1], traj[2][2], traj[0][3], traj[1][3], traj[2][3], 0])
        output_trajectories.append(output_trajectories)
        
    output_trajectories_df = pd.DataFrame(output_trajectories)
    output_trajectories_df.to_csv("output_trajectories.csv", index=False, header=False)
    