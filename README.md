# ECE-276A-Project3
## Visual-Inertial SLAM
This is the project 3 of the course UCSD ECE276A: Sensing & Estimation in Robotics.

<<<<<<< HEAD
## Implementations:

### Source files:
- **code/main.py**: Main function code for the project 3.
- **code/slam.py**: Function for Visual-Intertial SLAM.
- **code/slam.ipynb**: Notebook for testing the code.
- **code/slam_utils.py**: Utility sets of the of the SLAM.
- **code/visualize_utils.py**: Function for visualize the results.
    
### Usuage:
    python3 main.py


# UCSD ECE 276A Project 3: Visual-Inertial SLAM
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This project focuses on the implementation of visual-inertial simultaneous localization and mapping (SLAM) with an extended Kalman filter (EKF). You are provided with synchronized measurements from an inertial measurement unit (IMU) and a stereo camera as well as the intrinsic camera calibration and the extrinsic calibration between the two sensors, specifying the transformation from the left camera frame to the IMU frame.


<p align="center">
  <img src="https://github.com/lintsao/UCSD-ECE-276A-Project-2-Particle-Filter-SLAM/blob/main/gif/test_20.gif" alt="Project Image" width="400">
  <img src="https://github.com/lintsao/UCSD-ECE-276A-Project-2-Particle-Filter-SLAM/blob/main/gif/test_21.gif" alt="Project Image" width="400">
</p>
<p align="center">Here is a visual representation of our project. </p>

## To get started with the motion planning project, follow these steps:

1. Clone this repository:
  ```bash
  git clone https://github.com/lintsao/UCSD-ECE-276A-Project-2-Particle-Filter-SLAM.git
  cd UCSD-ECE-276A-Project-2-Particle-Filter-SLAM
  ```

2. Create a new virtual environment:
  ```bash
  python3 -m venv env
  source env/bin/activate  # For Unix/Linux
  ```

3. Install the required dependencies:
  ```bash
  pip3 install -r requirements.txt
  ```

4. You're ready to use the particle filter slam project!

## Usage

```
cd src
python3 main.py
```

## Source code description:
- **main.py**: Main function.
- **map.py**: Occupancy grid map related class and function.
- **motion.py**: Functions for motion model.
- **observation.py**: Functions for observation model and map correlation.
- **particle.py**: Particle class.
- **transfrom.py**: Transform helper (especially for lidar scan).
- **utils.py**: Functions for file loading, sync data, draw gif etc.
- **test.ipynb**: For testing.

or you could use **test.ipynb** to check the step-by-step implementation.

## Contributing
Contributions are welcome! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request.

=======
Implement visual-inertial simultaneous localization and mapping (SLAM) using an extended Kalman filter (EKF).

## Usage:
### Install package:
    pip3 install -r requirement.txt
### Run code:
    python3 main.py -d [dataset] --r [reduce factor for visual features] --w [w noise scale] --v [v noise scale]
### Example:
    python3 main.py --d 03 --r 4 --w 10e-6 --v 100
    python3 main.py --d 10 --r 4 --w 10e-6 --v 100


### Source code description:
- **code/main.py**: Main function.
- **code/mapping.py**: Functions for landmark mapping.
- **code/motion.py**: Functions for motion model.
- **code/observation.py**: Functions for observation model.
- **code/pr3_utils.py**: Functions provided for some transformation.
- **code/slam.ipynb**: For testing the process.
- **code/utils.py**: Functions for transformation and others.
- **code/visual_slam.py**: Functions for visual inertial slam.
- **code/visualization.py**: Functions for visualizing the trajectory and landmark mapping.
    
>>>>>>> 15f4356 (add new files)
