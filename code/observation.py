import numpy as np

def get_Ht(H_list, num_feature, isSLAM=False):
    if isSLAM:
        Nt = len(H_list)
        Ht = np.zeros([4*Nt, 3*num_feature+6])
        for i in range(Nt):
            j = H_list[i][0]      # landmark index
            H_obs  = H_list[i][1]
            H_pose = H_list[i][2]
            Ht[i*4:(i+1)*4, 3*j:3*(j+1)] = H_obs
            Ht[i*4:(i+1)*4, -6:] = H_pose
    else:
        Nt = len(H_list)
        Ht = np.zeros([4*Nt, 3*num_feature])
        for i in range(Nt):
            j = H_list[i][0]      # landmark index
            H = H_list[i][1]      # current Hij
            Ht[i*4:(i+1)*4,3*(j):3*(j+1)] = H
    return Ht

def get_Kt(cov, Ht, v_scale):
    V_noise  = np.eye(Ht.shape[0]) * v_scale
    inv_term = np.dot(Ht, np.dot(cov, Ht.T)) + V_noise
    Kt       = np.dot(np.dot(cov, Ht.T), np.linalg.inv(inv_term))
    return Kt
