#!/usr/bin/env python

import rosbag
import numpy as np
import teleop_utils as utils
import ipdb

# Open the rosbag file for reading
config = utils.load_config()
bag = rosbag.Bag(config['rosbag_file_path'])

# Access information about the bag (optional)
print("Bag information:")
print("Available Topics:", bag.get_type_and_topic_info().topics.keys())


# Extract and Process the messages
topic = '/arm/measured_cp'         # Change this str to the desired topic name
messages = bag.read_messages(topics=[topic])

# Initializing lists to add data
pose_list = []

for topic,msg,t in messages:
    # Get the times
    time = t.to_sec()
    
    # Get the poses
    pose = np.array([time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    pose_list.append(pose)


# Extract and Process the messages
topic = '/arm/button2'              # Change this str to the desired topic name
messages = bag.read_messages(topics=[topic])

# Initializing lists to add data
button2_events = []

for topic,msg,t in messages:
    # Get the time
    time = t.to_sec()
    # Get the button status
    button = msg.buttons[0]
    
    button2_events.append(np.array([time, button]))


# Extract and Process the messages
topic = '/arm/button1'              # Change this str to the desired topic name
messages = bag.read_messages(topics=[topic])

# Initializing lists to add data
button1_events = []

for topic,msg,t in messages:
    # Get the time
    time = t.to_sec()
    # Get the button status
    button = msg.buttons[0]
    
    button1_events.append(np.array([time, button]))


# Close the bag when done
bag.close()

pose_array = np.array(pose_list)
button1_clicks = np.array(button1_events)
button2_clicks = np.array(button2_events)

# Save the array to a npz file
filename = "data.npz"       # Provide a filename must end with ".npz"

np.savez(filename, button1 = button1_clicks, button2=button2_clicks, pose=pose_array)
