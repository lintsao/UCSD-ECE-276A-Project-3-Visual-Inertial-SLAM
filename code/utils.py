import numpy as np
from numpy.linalg import norm
import argparse


def parse_args():
    parser = argparse.ArgumentParser(description="SLAM")
    parser.add_argument('--d', default="03", type=str, help='dataset')
    parser.add_argument('--r', default=100, type=int, help='reduce feature factor')
    parser.add_argument('--w', default=10e-7, type=float, help='w_scale')
    parser.add_argument('--v', default=100, type=int, help='v_scale')

    return parser.parse_args()

def hat_map(vec):
    vec_hat = np.zeros((3,3))
    vec_hat[2,1] = vec[0]
    vec_hat[1,2] = -vec[0]
    vec_hat[2,0] = -vec[1]
    vec_hat[0,2] = vec[1]
    vec_hat[0,1] = -vec[2]
    vec_hat[1,0] = vec[2]
    return vec_hat

def adjoint(p, theta):
    p_hat     = hat_map(p)
    theta_hat = hat_map(theta)
    u_adjoint = np.zeros((6,6))
    u_adjoint[:3,:3] = theta_hat
    u_adjoint[3:,3:] = theta_hat
    u_adjoint[:3,3:] = p_hat
    return u_adjoint

def twist(p, theta):
    twist        = np.zeros((4,4))
    theta_hat    = hat_map(theta)
    twist[:3,:3] = theta_hat
    twist[:3,3]  = p
    return twist

def rodrigues_3(p, theta):
    u  = twist(p, theta) # get u in SE(3)
    u2 = np.dot(u, u)   # u^2
    u3 = np.dot(u, u2)  # u^3
    u2_coeff = (1 - np.cos(norm(theta))) / (norm(theta) ** 2)
    u3_coeff = (norm(theta) - np.sin(norm(theta))) / (np.power(norm(theta), 3))
    
    T = np.eye(4) + u + u2_coeff * u2 + u3_coeff * u3
    return T

def approx_rodrigues_3(p, theta):
    u  = twist(p,theta) # get u in SE(3)
    T = np.eye(4) + u
    return T

def rodrigues_6(p, theta):
    u = adjoint(p, theta) # get u in adjoint of SE(3)
    u2 = np.dot(u, u)     # u^2
    u3 = np.dot(u, u2)    # u^3
    u4 = np.dot(u, u3)    # u^4
    u_coeff  = (3*np.sin(norm(theta)) - norm(theta)*np.cos(norm(theta))) / (2 * norm(theta))
    u2_coeff = (4 - norm(theta)*np.sin(norm(theta)) - 4*np.cos(norm(theta))) / (2 * norm(theta)**2)
    u3_coeff = (np.sin(norm(theta)) - norm(theta)*np.cos(norm(theta))) / (2 * np.power(norm(theta),3))
    u4_coeff = (2 - norm(theta)*np.sin(norm(theta)) - 2*np.cos(norm(theta))) / (2 * np.power(norm(theta),4))
    
    T = np.eye(6) + u_coeff*u + u2_coeff*u2 + u3_coeff*u3 + u4_coeff*u4
    return T

def get_calibration(K, b):
    M   = np.vstack([K[:2], K[:2]])
    arr = np.array([0, 0, -K[0,0] * float(b), 0]).reshape((4, 1))
    M   = np.hstack([M, arr])
    return M

def pixel_to_world(p, i_T_w, o_T_i, K, b):
    uL, vL, uR, vR = p
    fsu = K[0,0]
    fsv = K[1,1]
    cu  = K[0,2]
    cv  = K[1,2]
    z   = (fsu*b) / (uL-uR) # Change to optical z.
    x   = z * (uL-cu) / fsu # Change to optical x.
    y   = z * (vL-cv) / fsv # Change to optical y.
    m_o = np.array([x,y,z,1]).reshape([4,1]) # Optical frame.
    m_i = np.dot(np.linalg.inv(o_T_i), m_o) # Optical => IMU frame.
    m_w = np.dot(np.linalg.inv(i_T_w), m_i) # IMU => World frame.
    return m_w

def projection(q):
    pi = q / q[2]
    return pi

def d_projection(q):
    dq = np.zeros((4,4))
    dq[0,0] = 1
    dq[1,1] = 1
    dq[0,2] = -q[0]/q[2]
    dq[1,2] = -q[1]/q[2]
    dq[3,2] = -q[3]/q[2]
    dq[3,3] = 1
    dq = dq / q[2]
    return dq

def circle(m):
    s      = m[:3]
    s_hat  = hat_map(s)
    result = np.hstack((np.eye(3), -s_hat))
    result = np.vstack((result, np.zeros((1,6))))
    return result

def roll(rad):
    return np.array([[1, 0, 0], [0, np.cos(rad), -np.sin(rad)],
                     [0, np.sin(rad), np.cos(rad)]])