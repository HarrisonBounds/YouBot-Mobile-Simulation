import modern_robotics as mr
import numpy as np

def FeedForwardControl(Tse, Tse_d, Tse_d_next, Kp, Ki, Je, dt):
    # Je = np.array([[0.030, -0.030, -0.030, 0.030, -0.985, 0, 0, 0, 0],
    #                 [0, 0, 0, 0, 0, -1, -1, -1, 0],
    #                 [-0.005, 0.005, 0.005, -0.005, 0.170, 0, 0, 0, 1],
    #                 [0.002, 0.002, 0.002, 0.002, 0, -0.240, -0.214, -0.218, 0],
    #                 [-0.024, 0.024, 0, 0, 0.221, 0, 0, 0, 0],
    #                 [0.012, 0.012, 0.012, 0.012, 0, -0.288, -0.135, 0, 0]])
    
    # print("Tse: ", Tse)
    # print("Tse_d: ", Tse_d)
    # print("Tse_d_next: ", Tse_d_next)
    
    
    total = np.zeros(6)
    Vd = (1/dt) * mr.MatrixLog6(mr.TransInv(Tse_d) @ Tse_d_next)
    # print("Vd before vec: ", Vd)
    Vd = mr.se3ToVec(Vd)
    
    d = mr.Adjoint(mr.TransInv(Tse) @ Tse_d) @ Vd
    
    X_err = mr.MatrixLog6(mr.TransInv(Tse) @ Tse_d)
    X_err = mr.se3ToVec(X_err)
    
    total += X_err * dt
    
    v = d + Kp @ X_err + Ki @ total
    
    speeds = np.linalg.pinv(Je) @ v
    
    # print("Vd: ", Vd)
    # print("test: ", d)
    # print("error: ", X_err)
    # print("V: ", v)
    # print("speeds: ", speeds)
    
    return speeds, X_err
    