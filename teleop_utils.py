#!/usr/bin/env python

import os
import sys
import configparser
import numpy as np
from spatialmath import *
import ipdb

def load_config():
    config_file_name = sys.argv[1]
    config_file_path = os.path.join('config',config_file_name+'.cfg')
    config = configparser.ConfigParser()
    config.read(config_file_path)
    # file paths
    rosbag_file_path = os.path.join('data_saved',config['General']['user_input_rosbag']+'.bag')
    user_input_data_path = os.path.join('data_saved',config_file_name+'_user_input_data.npz')
    teleop_command_data_path = os.path.join('data_saved',config_file_name+'_teleop_command_data.npz')
    # output
    config_data = {
        'name':config_file_name,
        'user_input_rosbag': rosbag_file_path,
        'user_input_data': user_input_data_path,
        'teleop_command_data': teleop_command_data_path,
    }
    return config_data

def ndarray_to_se3(nd_array: np.ndarray):
    return SE3([SE3(nd_array[i,:,:]) for i in range(nd_array.shape[0])])

def se3_to_ndarray(se3_array: SE3):
    return np.array([np.array(x) for x in se3_array])