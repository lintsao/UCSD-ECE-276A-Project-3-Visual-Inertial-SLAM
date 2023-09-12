import numpy as np
from utils import *
from motion import *
from observation import *
from pr3_utils import inversePose
from tqdm.auto import tqdm

def visual_slam(t, v, w, features, K, b, cam_T_imu, v_scale=100, w_scale=10e-5):
    # get time discretization
    tau = t[:,1:] - t[:,:-1]
    n = tau.shape[1]
    num_feature = features.shape[1]

    # initialize mu and covariance
    mu_imu  = np.eye(4)
    mu_obs  = np.zeros((4*num_feature, 1))
    cov     = np.eye(3 * num_feature + 6)
    
    # init with scale
    cov[:3*num_feature,:3*num_feature] *= 2
    cov[3*num_feature:,3*num_feature:] *= 1e-3
    W_noise = np.eye(6) * w_scale

    # poses
    imu_pose            = np.zeros((4,4,n+1)) # w_T_i
    inv_imu_pose        = np.zeros((4,4,n+1)) # i_T_w
    imu_pose[:,:,0]     = mu_imu
    inv_imu_pose[:,:,0] = np.linalg.inv(mu_imu)

    mu_hasinit  = np.zeros(num_feature)
    M       = get_calibration(K,b)
    P       = np.vstack([np.eye(3), np.zeros([1,3])]) # projection matrix
    P_block = np.tile(P, [num_feature, num_feature])

    for i in tqdm(range(n)):
        dt        = tau[:,i]          # time interval
        Ut        = mu_imu            # current inverse IMU pose
        feature   = features[:,:,i]   # current landmarks
        zt        = np.array([])      # to store zt
        zt_hat    = np.array([])      # to store zt_hat
        H_list = []                   # to store Hij
        observation_noise = np.random.randn() * np.sqrt(v_scale)
        for j in range(feature.shape[1]):
            # if is a valid feature
            if(feature[:,j] != np.array([-1,-1,-1,-1])).all():
                # check if has seen before
                # initialize if not seen before
                # update otherwise
                if(mu_hasinit[j] == 0):
                    m = pixel_to_world(feature[:,j], Ut, cam_T_imu, K, b)
                    mu_obs[4*j:4*(j+1)] = m
                    cov[3*j:3*(j+1), 3*j:3*(j+1)] = np.eye(3) * 1e-4
                    mu_hasinit[j] = 1     # mark as seen
                else:
                    mu_curr = mu_obs[4*j:4*(j+1)]
                    q1      = np.dot(cam_T_imu, np.dot(Ut, mu_curr))
                    q2      = circle(np.dot(Ut, mu_curr))
                    zt      = np.concatenate((zt, feature[:,j]+observation_noise), axis=None)
                    zt_hat  = np.concatenate((zt_hat, np.dot(M, projection(q1))), axis=None)
                    # compute H
                    H_obs   = ((M.dot(d_projection(q1))).dot(cam_T_imu).dot(Ut)).dot(P)
                    H_pose  = (M.dot(d_projection(q1))).dot(cam_T_imu).dot(q2)
                    H_list.append((j, H_obs, H_pose))

        Nt     = len(H_list)
        zt     = zt.reshape([4*Nt,1])
        zt_hat = zt_hat.reshape([4*Nt,1])
        Ht     = get_Ht(H_list, num_feature, isSLAM=True)
        Kt     = get_Kt(cov, Ht, v_scale)

        # update mean and cov
        Kt_obs  = Kt[:-6,:]  # 3M x 4Nt
        Kt_pose = Kt[-6:, :] #  6 x 4Nt

        p      = np.dot(Kt_pose, zt-zt_hat)[:3].flatten()
        theta  = np.dot(Kt_pose, zt-zt_hat)[-3:].flatten()

        mu_obs = mu_obs + P_block.dot(Kt_obs.dot(zt-zt_hat))
        
        mu_imu = np.dot(approx_rodrigues_3(p, theta), mu_imu)
        cov    = np.dot((np.eye(3*num_feature+6) - np.dot(Kt,Ht)), cov)
        
        # store imu pose
        inv_imu_pose[:,:,i+1] = mu_imu
        imu_pose[:,:,i+1] = np.linalg.inv(mu_imu)

        # predict mean and cov
        noise  = np.random.randn() * w_scale
        v_curr = v[:,i] + noise
        w_curr = w[:,i] + noise
        mu_imu = mu_predict(mu_imu, v_curr, w_curr, dt)
        cov[-6:,-6:]  = cov_predict(cov[-6:,-6:], v_curr, w_curr, dt, W_noise)
        
    mu_obs = mu_obs.reshape((num_feature, 4))
    return inv_imu_pose, imu_pose, mu_obs