#!/usr/bin/env python

# import rosbag
import numpy as np
import teleop_utils as utils
from pathlib import Path
from rosbags.highlevel import AnyReader

def extract(bagfile):
    # Initializing lists to add data
    pose_list = []
    button1_list = []
    button2_list = []

    # create reader instance and open for reading
    with AnyReader([Path(bagfile)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/arm/measured_cp']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000)
            pose = np.array([time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            pose_list.append(pose)
        
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

    button1_array = np.array(button1_list)
    button2_array = np.array(button2_list)
    pose_array = np.array(pose_list)

    filename = bagfile[:-4] + "_extracted.npz"   # Saves the data to a npz file of similar name
    np.savez(filename, button1=button1_array, button2=button2_array, pose=pose_array)
    return filename 
    
if __name__ == '__main__':
    config = utils.load_config()
    bagfile = config['rosbag_file_path']
    x = extract(bagfile)
    print("File Saved As: ", x)