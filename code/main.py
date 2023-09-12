import numpy as np
from pr3_utils import load_data
from utils import *
from motion import *
from observation import *
from mapping import *
from visual_slam import *
from visualization import *


if __name__ == '__main__':

	# set parameters
	args = parse_args()
	dataset = args.d
	reduce_factor = args.r
	w_scale = args.w
	v_scale = args.v
	filename = f"../data/{dataset}.npz"

	save_path_traj = f"../fig/trajectory_2d_{dataset}_{reduce_factor}.png"
	save_path_mapping = f"../fig/mapping_{dataset}_{reduce_factor}.png"
	save_path_slam = f"../fig/slam_{dataset}_{reduce_factor}.png"
	save_path_compare = f"../fig/compare_{dataset}_{reduce_factor}.png"

	# Load the measurements
	t, features, linear_velocity, angular_velocity, K, b, imu_T_cam = load_data(filename)
	
	# transform for further usage
	cam_T_imu = inversePose(imu_T_cam)
	cam_T_imu[:3,:3] = roll(np.pi) @ cam_T_imu[:3,:3]

	# downsample features
	selected_feats = np.zeros((4, int(features.shape[1] / reduce_factor) + 1, features.shape[2]))
	for i, idx in enumerate(range(0, int(features.shape[1]), reduce_factor)):
		selected_feats[:,i,:] = features[:,idx,:]
	print(f"Using {selected_feats.shape[1]} / {features.shape[1]} of features")

	# (a) IMU Localization via EKF Prediction
	print("Start IMU Localization ...")
	i_T_w, w_T_i = motion_model_prediction(t, linear_velocity, angular_velocity, w_scale)
	visualize_trajectory_2d(w_T_i, save_path_traj, show_ori=True)

	# (b) Landmark Mapping via EKF Update
	print("Start Landmark Mapping ...")
	landmarks = landmark_mapping(selected_feats, i_T_w, K, b, cam_T_imu, v_scale)
	visualize(w_T_i, landmarks, save_path_mapping)

	# (c) Visual-Inertial SLAM
	print("Start Visual-Inertial SLAM ...")
	slam_iTw, slam_wTi, slam_landmarks = visual_slam(t, linear_velocity, angular_velocity, selected_feats, K, b, cam_T_imu, v_scale, w_scale)
	visualize(slam_wTi, slam_landmarks, save_path_slam)

	# You can use the function below to visualize the robot pose over time
	# visualize_trajectory_2d(world_T_imu, show_ori = True)
	# compare result of dead-reckoning and visual slam
	combine_visualize(w_T_i, slam_wTi, landmarks, slam_landmarks, save_path_compare, show_plot=True)


