# teleop_python_utils/README.md

## 3ds_rosbag_extract.py

To extract rostopic messages from a .bag file containing topics published by these [ROS drivers](https://github.com/jhu-saw/sawSensablePhantom) into a .npz file as NumPy Arrays.

Sample bag files are provided in this repo. [data_saved](teleop_python_utils/data_saved)

The .npz file will get saved in the location of your python environment.

Edit `config/user_input1.cfg` to reflect the bagfile of interest: 

```
rosbag_file_name = 2023-06-29-13-18-02
``` 

To run the script, in your terminal:
```Shell
python 3ds_rosbag_extract.py user_input1
```
## Set up development environment

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