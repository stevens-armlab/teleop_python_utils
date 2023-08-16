# teleop_python_utils/README.md

### Set up development environment

Python 3.10 is required, however, since it is no longer supported by 18.04, we need to build it from source.
```
  # [prerequisite]
  # Python 3.10 is not available under 18.04
  # Build Python 3.10 from source
  # https://ubuntuhandbook.org/index.php/2021/10/compile-install-python-3-10-ubuntu/
```

```
  # create venv with python3.10
  rm -rf ~/venv/py_kin_py310
  python3.10 -m venv ~/venv/py_kin_py310
  source ~/venv/py_kin_py310/bin/activate

  # Install using PyPI:
  python -m pip install --upgrade pip setuptools wheel
  mkdir -p ~/tmpPythonInstall
  TMPDIR=~/tmpPythonInstall python -m pip install roboticstoolbox-python
  python -m pip install pyqt5 # Must install GUI backend for plotting
  python -m pip install ipdb
  python -m pip install rosbags

```

### How to use this codebase
#### 0. Collect the data
The raw data of 3ds Touch hatpic device can be collected by following https://github.com/jhu-cisst-external/3ds-touch-openhaptics

#### 0. What to expect
There are a few steps to compute the issued teleop commands based on raw user input data collected:
1. ```3ds_rosbag_extract.py``` is used to extract user input data, from rosbag to numpy data files for better accessability.
2. ```rel_pose_computation.py``` is used to compute teleoperation command using the numpy user input data
#### 1. Create a configuration file under [/config](teleop_python_utils/config)
To start, one need to make a copy of the example config file ```config/traj1.cfg```, and edit it to reflect:

1. The user input rosbag file of interest: 
    ```
    rosbag_file_name = 2023-06-29-13-18-02
    ``` 
2. The scaling factor for teleoperation:
    ```
    scaling_factor = 1
    ```
3. The world rotation matrix (defines the viewers perspective) :
    ```
    world_rot_mat = [[1,0,0],[0,1,0],[0,0,1]]
    ```
    Note: If the world rotation matrix is invalid, it will default to the identity matrix

#### 2. Run 3ds_rosbag_extract.py
Run the following in a terminal:
```Shell
python 3ds_rosbag_extract.py traj1
```
Then, ```traj1_user_input_data.npz``` will be saved under [data_saved](teleop_python_utils/data_saved)

**Explain**: we extract rostopic messages from a .bag file containing topics published by these [ROS drivers](https://github.com/jhu-saw/sawSensablePhantom) into a .npz file as NumPy Arrays. Sample bag files are provided in this repo. [data_saved](teleop_python_utils/data_saved)

#### 3. Run rel_pose_computation.py
Run the following in a terminal:
```Shell
python rel_pose_computation.py traj1
```
Then, all issued teleop commands will be printed in the terminal.

