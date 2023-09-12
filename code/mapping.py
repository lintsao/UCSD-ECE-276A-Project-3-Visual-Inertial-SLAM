import numpy as np
from utils import *
from tqdm.auto import tqdm

def landmark_mapping(features, i_T_w, K, b, cam_T_imu, v_scale=100):
    num_feature = features.shape[1]
    mu_hasinit  = np.zeros(num_feature)
    mu      = np.zeros((4 * num_feature, 1))
    cov     = np.eye(3 * num_feature)
    M       = get_calibration(K, b) # calibration matrix 
    P       = np.vstack([np.eye(3), np.zeros([1,3])]) # projection matrix
    P_block = np.tile(P, [num_feature, num_feature])

    for i in tqdm(range(features.shape[2])):
        Ut        = i_T_w[:,:,i]      # current inverse IMU pose
        feature   = features[:,:,i]   # current landmarks
        zt        = np.array([])      # to store zt
        zt_hat    = np.array([])      # to store zt_hat
        H_list = []                   # to store Hij
        observation_noise = np.random.randn() * np.sqrt(v_scale)
        for j in range(feature.shape[1]):
            # if is a valid feature
            if(feature[:, j] != np.array([-1, -1, -1, -1])).all():
                # check if has seen before
                # initialize if not seen before
                # update otherwise
                if(mu_hasinit[j] == 0):
                    m = pixel_to_world(feature[:,j], Ut, cam_T_imu, K, b) # get the world frame form the pixel frame.
                    mu[4 * j:4 * (j+1)] = m
                    cov[3 * j:3 * (j+1), 3 * j:3 * (j+1)] = np.eye(3) * 1e-3
                    mu_hasinit[j] = 1     # mark as seen
                else:
                    mu_curr = mu[4*j:4*(j+1)]
                    q       = np.dot(cam_T_imu, np.dot(Ut, mu_curr)) # get the optical frame form the world frame.
                    zt      = np.concatenate((zt, feature[:,j]+observation_noise), axis=None)
                    zt_hat  = np.concatenate((zt_hat, np.dot(M, projection(q))), axis=None)
                    # compute H_ij
                    H       = ((M.dot(d_projection(q))).dot(cam_T_imu).dot(Ut)).dot(P)
                    H_list.append((j, H))

        Nt     = len(H_list)
        zt     = zt.reshape([4*Nt,1])
        zt_hat = zt_hat.reshape([4*Nt,1])
        Ht     = get_Ht(H_list, num_feature)
        Kt     = get_Kt(cov, Ht, v_scale)

        # update mu and cov
        mu = mu + P_block.dot(Kt.dot(zt-zt_hat))
        cov = np.dot((np.eye(3*num_feature) - np.dot(Kt,Ht)),cov)

    landmarks = mu.reshape([num_feature,4])
    return landmarks

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

def get_Kt(cov, Ht, v):
    V_noise  = np.eye(Ht.shape[0]) * v
    inv_term = np.dot(Ht, np.dot(cov, Ht.T)) + V_noise
    Kt       = np.dot(np.dot(cov, Ht.T), np.linalg.inv(inv_term))
    return Kt