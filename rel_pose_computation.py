#!/usr/bin/env python

import numpy as np
import teleop_utils as utils
from spatialmath import *
import matplotlib.pyplot as plt
import ipdb

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

def filter_poses_by_time(time_stamps, time_range, user_input_traj):
    # Find the indices where the time values in the poses array fall within the given range
    index_within_range = np.where(
        (time_stamps >= time_range[0]) & (time_stamps <= time_range[1]))
    return user_input_traj[index_within_range[0],:,:]
   
def rel_pose_traj(user_input_traj):
    # get an array of relative positions
    start_pose = user_input_traj[0]
    return SE3([start_pose.inv()*pose for pose in user_input_traj])

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

    pose_msg:
    [time, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation]
    
    user_input_traj:
    [   [SE3() at time_0]
        [SE3() at time_1]
        [SE3() at time_2]
            .
            .
            .
        [SE3() at time_n]

    """
    config = utils.load_config()
    data = np.load(config['user_input_data'])

    button_not_used = data['button1']
    button_enable = data['button2']
    pose_msg = data['pose_msg']
    user_input_traj = data['user_input_traj']

    anchors = anchor_events(button_enable)
    selected_anchor_range = anchors[0]
    total_time = selected_anchor_range[1] - selected_anchor_range[0]

    user_input_traj_fltr = filter_poses_by_time(
        time_stamps = pose_msg[:,0], time_range = selected_anchor_range, user_input_traj = user_input_traj)

    user_input_traj_fltr = SE3(
        [SE3(user_input_traj_fltr[i,:,:]) for i in range(user_input_traj_fltr.shape[0])]
        ) # this forced conversion using SE3(list of SE3 instances) is necessary to enable user_input_traj.plot() 

    rel_traj = rel_pose_traj(user_input_traj_fltr) 

    for idx, pose_i in enumerate(rel_traj):
        print(f'via point {idx}')
        print(pose_i)

    # Animation 1: User Input
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.title('User Input Traj')
    user_input_traj_fltr[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    user_input_traj_fltr.animate(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.show(block=False)
    input('Animation displayed')

    # Animation 2: User Input (Relative)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.title('User Input Traj (Relative)')
    rel_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    rel_traj.animate(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))

    plt.show(block=False)
    input('Animation displayed (Relative)')

    # Comparison: User Input vs. Relative
    time_to_view = float(input(
        f'Total time = {total_time}, select a time view: '))
    index_to_view = int (time_to_view/total_time*len(rel_traj))
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    user_input_traj_fltr[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    user_input_traj_fltr[index_to_view].plot(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj')
    
    ax = fig.add_subplot(122, projection='3d')
    rel_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    rel_traj[index_to_view].plot(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj (Relative)')

    plt.show(block=False)
    input('User Input Traj displayed')
