import modern_robotics as mr
import numpy as np
from feedforward_control import FeedForwardControl
from trajectory_generation import Trajectory_Generation
from next_state import Next_State
import pandas as pd

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

Tse = np.array([[0.170, 0, 0.985, 0.387],
                [0, 1, 0, 0],
                [-0.985, 0, 0.170, 0.570],
                [0, 0, 0, 1]])

Tse_d = np.array([[0, 0, 1, 0.5],
                [0, 1, 0, 0],
                [-1, 0, 0, 0.5],
                [0, 0, 0, 1]]) 

Tse_d_next = np.array([[0, 0, 1, 0.6],
                  [0, 1, 0, 0],
                  [-1, 0, 0, 0.3],
                  [0, 0, 0, 1]]) 

Kp = np.zeros((6, 6))
Ki = np.zeros((6, 6))

current_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
current_configs = []
x_errs = []

n = 2100

Trajectory_Generation(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k=1, gripper_state=False)

for i in range(n - 1):
    wheel_and_arm_joint_speeds, x_err = FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt=0.01)
    current_config = Next_State(current_config, wheel_and_arm_joint_speeds, angular_range=50)
    current_configs.append(current_config.copy())
    x_errs.append(x_err)
    
current_config_df = pd.DataFrame(current_configs)
current_config_df.to_csv("next_state.csv", index=False, header=False)
