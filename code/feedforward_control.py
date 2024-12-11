import modern_robotics as mr
import numpy as np

def FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, Je, dt):
    """
    Computes the feedforward control for a mobile manipulator based on the current end-effector 
    pose, desired poses, and control gains. The function calculates joint speeds and the error 
    in the end-effector pose using a feedforward and feedback control law.

    Args:
        Tse (np.ndarray): Current end-effector pose as a 4x4 transformation matrix.
        Tse_d (np.ndarray): Desired end-effector pose at the current timestep as a 4x4 matrix.
        Tse_d_next (np.ndarray): Desired end-effector pose at the next timestep as a 4x4 matrix.
        Kp (np.ndarray): 6x6 proportional gain matrix for feedback control.
        Ki (np.ndarray): 6x6 integral gain matrix for feedback control.
        Je (np.ndarray): End-effector Jacobian as a 6xN matrix.
        dt (float): Time step for numerical differentiation.

    Returns:
        np.ndarray: Joint speeds as an N-element vector.
        np.ndarray: End-effector error as a 6-element vector in se(3).
    """
    # Initialize the integral of the error as zero
    total = np.zeros(6)

    # Compute desired end-effector twist using the difference between desired poses
    Vd = (1 / dt) * mr.MatrixLog6(mr.TransInv(Tse_d) @ Tse_d_next)  # Compute relative motion
    Vd = mr.se3ToVec(Vd)  # Convert to a 6D twist vector

    # Compute the feedforward component of the control
    d = mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd

    # Compute the error in the end-effector pose
    X_err = mr.MatrixLog6(mr.TransInv(Tse) @ Tse_d)  # Pose error in matrix form
    X_err = mr.se3ToVec(X_err)  # Convert error to a 6D vector

    # Accumulate the integral of the error
    total += X_err * dt

    # Compute the feedforward + feedback control law
    v = d + Kp @ X_err + Ki @ total

    # Compute the joint speeds using the pseudo-inverse of the Jacobian
    speeds = np.linalg.pinv(Je) @ v

    return speeds, X_err
