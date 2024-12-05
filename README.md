# YouBot-Mobile-Simulation

## Milestone 2

The code in Milestone 2 provides a trajectory for the end effector to move to the initial cube position and drop it off in its final position. The function used to perform these actions is call **Trajectory_Generation**. It takes several transforms as inputs, such as:

- Tse_initial: np.ndarray
- Tsc_initial: np.ndarray
- Tsc_final: np.ndarray
- Tce_grasp: np.ndarray
- Tce_standoff: np.ndarray
- k: int
- gripper_state: bool

This function calculates everything according to the **{S}** frame. It then uses the **ScrewTrajectory** function in the Modern Robotics Package to find the screw trajectory between the paramets (X_start, X_end). After that, the trajectories are formatted to be used by the end-effector in Coppelia Sim, and saved to an output csv file called **output_trajectories**.

Example run(On a Linux OS):

```
python3 trajectory_generation.py
```

This will rewrite the ouptut_trajcetories.csv and be ready for use in Coppelia Sim.