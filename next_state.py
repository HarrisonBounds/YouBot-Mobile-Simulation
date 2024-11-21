import modern_robotics as mr
import numpy as np
import pandas as pd

dt = 0.01
num_iters = 100

current_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
controls = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
angular_range = 12.3


def Next_State(current_config, controls, dt, angular_range):
    
    #Update Arm joint angles
    mr.EulerStep()
    clipped_controls = np.clip(controls, -angular_range, angular_range)
    current_config.to_csv("youBot_configuration.csv", index=False, header=False)
    return

for i in range(num_iters):
    Next_State()