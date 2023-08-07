#!/usr/bin/env python

import os
import sys
import configparser

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