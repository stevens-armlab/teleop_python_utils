#!/usr/bin/env python

import numpy as np

def total_time(pose_array):
    """
    Returns the time range of that pose array
    Preconditions: The Numpy pose_array is sorted chronologically
    The pose_array's first element is the time
    """
    return pose_array[len(pose_array)-1][0] - pose_array[0][0]

def anchor_events(button):
    """
    Returns each anchor event as an array where 
    each element of array is [time_press, time_release]
    """
    anchors = []
    for i in range(len(button) - 1):
        if i % 2 == 1:
            anchors.append([button[i,0], button[i+1,0]])
    return anchors

def filter_poses_by_time(pose_array, time_range):
    # Find the indices where the time values in the poses array fall within the given range
    mask = (pose_array[:, 0] >= time_range[0]) & (pose_array[:, 0] <= time_range[1])
    return pose_array[mask]
   
def quaternion_multiply(q1, q2):
    """
    Returns the quaternion multiplication of q1 * q2
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x_res = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_res = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_res = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w_res = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return np.array([x_res, y_res, z_res, w_res])

def rel_pose_traj(pose_array):
    """
    pose_array: an array of poses. Each pose has 8 elements.
    [time, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation]
    
    Returns a NumPy array of poses relative to the first element
    """
    # get an array of relative positions
    i = 0
    start_pose = pose_array[0]
    start_orient_inv = np.array([-pose_array[0, 4], -pose_array[0, 5], -pose_array[0, 6], pose_array[0, 7]])
    pose_list = []

    while (i < len(pose_array)):
        rel_pstn = pose_array[i,0:4] - start_pose[0:4]
        rel_orient = quaternion_multiply(pose_array[i, 4:8], start_orient_inv)
        rel_pose = np.hstack([rel_pstn, rel_orient])
        pose_list.append(rel_pose)
        i = i + 1

    return np.array(pose_list)

if __name__ == '__main__':
    """ 
    Load the data from an npz file where:
    
    button1 and button2:
    [   [time_0 event_0]
        [time_1 event_1]
                .      
                .      
                .      
        [time_n event_n]    ]

    pose:
    [time, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation]
        
    """
    data = np.load('data.npz')

    button1 = data['button1']
    button2 = data['button2']
    pose = data['pose']

    anchors = anchor_events(button2)
    fltr_poses = filter_poses_by_time(pose, anchors[0])
    
    traj = rel_pose_traj(fltr_poses) 
    print(traj[0])