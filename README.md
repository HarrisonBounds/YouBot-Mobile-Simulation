# Robotic Arm Trajectory Generation and Control - YouBot Simulation

## Table of Contents

- [Overview](#overview)
- [Dependencies](#dependencies)
- [Components](#components)
  - [Trajectory Generation](#trajectory-generation)
  - [Feedforward Control](#feedforward-control)
  - [Next State Calculation](#next-state-calculation)
  - [Plotting](#plotting)
- [Usage](#usage)
  - [Running the Code](#running-the-code)
  - [Error Plot](#error-plot)


## Overview

This project is focused on simulating the motion of a robotic arm with a gripper. The robotic arm is modeled with a series of transformations and forward kinematics. The system generates trajectories for the arm's motion, computes control inputs to follow the trajectories, and uses inverse kinematics to move the arm and gripper to the desired positions. The system also includes error tracking and generates plots of the error for performance evaluation.

Key functionalities of the code include:

- **Trajectory Generation**: Generates desired end-effector trajectories based on initial and final configurations.
- **Feedforward Control**: Computes control inputs to minimize errors in following the desired trajectories.
- **Inverse Kinematics**: Computes the configuration of the robot's joints based on the desired end-effector position.
- **State Propagation**: Simulates the robot’s motion through discrete time steps.
- **Error Plotting**: Plots the errors between the current and desired configurations of the robot.

## Dependencies

The code depends on the following Python libraries:

- "numpy": For matrix manipulations and numerical operations.
- "modern_robotics": For advanced robotics math operations (inverse kinematics, Jacobians, etc.).
- "pandas": For handling and exporting data to CSV files.
- "matplotlib" (implicitly used for plotting errors).

## Components

### Trajectory Generation

The **Trajectory_Generation** module creates a series of target poses (SE(3) matrices) for the robot to reach. It takes as input the initial and final configurations of the robot and the desired gripper states, generating the movement trajectory in discrete steps.

### Feedforward Control

The **FeedForwardControl** module computes control inputs (wheel and arm joint speeds) to minimize the error between the current robot pose and the desired pose. It uses a proportional-integral (PI) control strategy, and the errors are tracked over time to evaluate the control performance.

### Next State Calculation

The **Next_State** module updates the robot’s state at each time step, taking into account the wheel and arm joint speeds. The robot’s position and configuration evolve over time based on these control inputs.

### Plotting

The **Plot** module visualizes the trajectory tracking performance by plotting the errors between the current robot state and the desired state over time. It provides a way to assess how well the robot is following the planned trajectory.

## Usage

### Running the Code

1. Make sure that you have all the dependencies installed.
2. Modify the initial and final configurations of the robot and gripper according to your task.
3. Run the main script. This will execute the trajectory generation, feedforward control, and update the robot’s state.


### Error Plot

Once the execution is complete, an error plot will be displayed, showing how the robot's pose errors evolve over time. This helps to visualize the control performance and trajectory tracking.

