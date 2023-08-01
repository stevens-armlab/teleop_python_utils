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
