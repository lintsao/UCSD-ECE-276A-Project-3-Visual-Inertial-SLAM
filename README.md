# UCSD ECE 276A Project 3: Visual-Inertial SLAM
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This project focuses on the implementation of visual-inertial simultaneous localization and mapping (SLAM) with an extended Kalman filter (EKF). You are provided with synchronized measurements from an inertial measurement unit (IMU) and a stereo camera as well as the intrinsic camera calibration and the extrinsic calibration between the two sensors, specifying the transformation from the left camera frame to the IMU frame.


<p align="center">
  <img src="https://github.com/lintsao/UCSD-ECE-276A-Project-3-Visual-Inertial-SLAM/blob/main/fig/slam_03_100.png" alt="Project Image" width="400">
</p>
<p align="center">Here is a visual representation of our project. </p>

## To get started with the motion planning project, follow these steps:

1. Clone this repository:
  ```bash
  git clone https://github.com/lintsao/UCSD-ECE-276A-Project-3-Visual-Inertial-SLAM.git
  cd UCSD-ECE-276A-Project-3-Visual-Inertial-SLAM
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

4. You're ready to use the Visual-Inertial-SLAM project!

## Usage

```
cd code
python3 main.py -d [dataset] --r [reduce factor for visual features] --w [w noise scale] --v [v noise scale]
```
Example
```
python3 main.py --d 03 --r 4 --w 10e-6 --v 100
python3 main.py --d 10 --r 4 --w 10e-6 --v 100
```

### Source code description:
- **main.py**: Main function.
- **mapping.py**: Functions for landmark mapping.
- **motion.py**: Functions for motion model.
- **observation.py**: Functions for observation model.
- **pr3_utils.py**: Functions provided for some transformation.
- **slam.ipynb**: For testing the process.
- **utils.py**: Functions for transformation and others.
- **visual_slam.py**: Functions for visual-inertial slam.
- **visualization.py**: Functions for visualizing the trajectory and landmark mapping.

or you could use **slam.ipynb** to check the step-by-step implementation.

## Contributing
Contributions are welcome! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request.
