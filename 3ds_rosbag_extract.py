#!/usr/bin/env python

import rosbag
import numpy as np

# Open the rosbag file for reading
bag = rosbag.Bag('/home/armlab1/bagfiles/2023-06-29-13-18-02.bag')

# Access information about the bag (optional)
print("Bag information:")
print("Available Topics:", bag.get_type_and_topic_info().topics.keys())


# Extract and Process the messages
topic = '/arm/measured_cp'         # Change this str to the desired topic name
messages = bag.read_messages(topics=[topic])

# Initializing lists to add data
time_list = []
pose_list = []

for topic,msg,t in messages:
    # Get the times
    time_list.append(t.to_sec())

    # Get the poses
    positions = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    orientations = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    pose = np.array([positions, orientations])
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

time_array = np.array(time_list)
pose_array = np.array(pose_list)
button1_clicks = np.array(button1_events)
button2_clicks = np.array(button2_events)

# Save the array to a npz file
filename = "data.npz"       # Provide a filename must end with ".npz"

np.savez(filename, time=time_array, pose=pose_array, button1 = button1_clicks, button2=button2_clicks)
