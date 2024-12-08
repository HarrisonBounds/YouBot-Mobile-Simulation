import modern_robotics as mr
import numpy as np

robot_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])

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

S =  np.array([[0, 0, 0, 0, 0],
              [0, -1, -1, -1, 0],
              [1, 0, 0, 0, 1],
              [0, -0.5076, -0.3526, -0.2176, 0],
              [0.033, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]])

# Je = np.array([[0.030, -0.030, -0.030, 0.030, -0.985, 0, 0, 0, 0],
#                    [0, 0, 0, 0, 0, -1, -1, -1, 0],
#                    [-0.005, 0.005, 0.005, -0.005, 0.170, 0, 0, 0, 1],
#                    [0.002, 0.002, 0.002, 0.002, 0, -0.240, -0.214, -0.218, 0],
#                    [-0.024, 0.024, 0, 0, 0.221, 0, 0, 0, 0],
#                    [0.012, 0.012, 0.012, 0.012, 0, -2.88, -0.135, 0, 0]])

Je = mr.JacobianSpace(S, robot_config)

Kp = np.zeros((6, 6))
Ki = np.zeros((6, 6))
dt = 0.01 


def FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt):
    
    
    total = np.zeros(6)
    Vd = (1/dt) * mr.MatrixLog6(mr.TransInv(Tse_d) @ Tse_d_next)
    Vd = mr.se3ToVec(Vd)
    
    d = mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd
    
    X_err = mr.MatrixLog6(mr.TransInv(Tse) @ Tse_d)
    X_err = mr.se3ToVec(X_err)
    
    total += X_err * dt
    
    V = d + Kp @ X_err + Ki @ total
    
    speeds = np.linalg.pinv(Je) @ V
    
    # print("Vd: ", Vd)
    # print("test: ", d)
    # print("error: ", X_err)
    # print("V: ", V)
    # print("speeds: ", speeds)
    
    return speeds, X_err
    
    
    
FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt)