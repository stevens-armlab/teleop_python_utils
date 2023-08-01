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
    rosbag_file_path = os.path.join('data_saved',config['General']['rosbag_file_name']+'.bag')
    config_data = {
        'rosbag_file_path': rosbag_file_path,
    }
    return config_data