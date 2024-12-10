import modern_robotics as mr
import numpy as np

def Next_State(current_config, controls, dt, angular_range):
    l = 0.235
    w = 0.15
    r = 0.0475
    new_config = np.zeros(13)
    
    controls = np.clip(controls, -angular_range, angular_range)
    
    #Update Arm Angles
    for i in range(5): #Number of joints for arm
        new_config[i+3] = current_config[i+3] + controls[i+4] * dt
    
    #Update wheel angles
    for i in range(4): #Number of wheels
        new_config[i+8] = current_config[i+8] + controls[i] * dt
        
    phi = current_config[0]
    x = current_config[1]
    y = current_config[2]
    
    m = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1 ,1],
                    [-1, 1, -1, 1]])
    
    
    V_b = (r / 4) * m @ np.array([new_config[8]-current_config[8], new_config[9]-current_config[9], new_config[10]-current_config[10], new_config[11]-current_config[11]])
    
    V_b6 = np.array([0, 0, V_b[0], V_b[1], V_b[2], 0])
    
    V_b6 = mr.VecTose3(V_b6)
    
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    
    Tsbk1 = Tsb @ mr.MatrixExp6(V_b6)
    
    #Update Odometry
    new_config[0] = np.arccos(Tsbk1[0][0])
    new_config[1] = Tsbk1[0][3]
    new_config[2] = Tsbk1[1][3]
        
    return new_config
