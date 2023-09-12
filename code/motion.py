import numpy as np
from utils import *
from tqdm.auto import tqdm

def motion_model_prediction(t, v, w, w_scale=10e-7):
    # get time discretization
    tau = t[:,1:] - t[:,:-1]
    n = tau.shape[1]

    # initialize mu, covariance, and noise
    mu  = np.eye(4)
    cov = np.eye(6)
    W_noise = np.eye(6) * w_scale

    # poses
    imu_pose = np.zeros((4, 4, n + 1))      # w_T_i 4*4 coordinate
    inv_imu_pose = np.zeros((4, 4, n + 1))  # i_T_w
    imu_pose[:,:,0] = mu                    # initialize
    inv_imu_pose[:,:,0] = np.linalg.inv(mu) # initialize

    for i in tqdm(range(n)):
        dt     = tau[:,i]
        linear_noise  = np.random.randn(3) * w_scale # x, y, z
        angular_noise = np.random.randn(3) * w_scale # x, y, z
        v_curr = v[:,i] + linear_noise
        w_curr = w[:,i] + angular_noise
        mu     = mu_predict(mu, v_curr, w_curr, dt)
        # no need of cov

        # cov    = cov_predict(cov, v_curr, w_curr, dt, W_noise)
        # v_curr *= -dt
        # w_curr *= -dt
        # u_curr = np.hstack((v_curr, w_curr))
        # mu = axangle2pose(u_curr)
        # cov = pose2adpose(mu)
        inv_imu_pose[:,:,i+1] = mu
        imu_pose[:,:,i+1] = np.linalg.inv(mu)
        
    return inv_imu_pose, imu_pose
    # return imu_pose

def mu_predict(mu, v, w, dt):
    p       = -dt * v
    theta   = -dt * w
    mu_pred = np.dot(rodrigues_3(p, theta), mu) # Rodrigues transformation is a method used to represent a rotation in three-dimensional space by specifying a rotation axis and an angle of rotation as a matrix. This representation is commonly employed to perform rotation operations in three-dimensional space.
    return mu_pred

def cov_predict(cov, v, w, dt, noise):
    p        = -dt * v
    theta    = -dt * w
    cov_pred = np.dot(rodrigues_6(p, theta), cov)
    cov_pred = np.dot(cov_pred, rodrigues_6(p, theta).T)
    cov_pred = cov_pred + noise
    return cov_pred