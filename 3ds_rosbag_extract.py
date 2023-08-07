#!/usr/bin/env python

# import rosbag
import numpy as np
import teleop_utils as utils
from pathlib import Path
from rosbags.highlevel import AnyReader
from spatialmath import *
import os
import ipdb

def extract(config):
    # Initializing lists to add data
    pose_msg_list = []
    user_input_traj = []
    button1_list = []
    button2_list = []
    # create reader instance and open for reading
    with AnyReader([Path(config['user_input_rosbag'])]) as reader:
        connections = [x for x in reader.connections if x.topic == '/arm/measured_cp']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # reading raw pose messages
            msg = reader.deserialize(rawdata, connection.msgtype)
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)
            pose_msg = np.array([time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            pose_msg_list.append(pose_msg)

            pose_SE3 = SE3(SO3(UnitQuaternion(
                s = msg.pose.orientation.w, 
                v = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]).R))
            pose_SE3.t = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            user_input_traj.append(pose_SE3)
                
        connections = [x for x in reader.connections if x.topic == '/arm/button1']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)
            button1_list.append(np.array([time, msg.buttons[0]]))

        connections = [x for x in reader.connections if x.topic == '/arm/button2']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)
            button2_list.append(np.array([time, msg.buttons[0]]))

    button1_msg_array = np.array(button1_list)
    button2_msg_array = np.array(button2_list)
    pose_msg_array = np.array(pose_msg_list)

    np.savez(config['user_input_data'], 
             button1=button1_msg_array, 
             button2=button2_msg_array, 
             pose_msg=pose_msg_array, 
             user_input_traj=user_input_traj)
    return config['user_input_data']
    
if __name__ == '__main__':
    config = utils.load_config()
    x = extract(config)
    print("File Saved As: ", x)