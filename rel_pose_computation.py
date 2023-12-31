#!/usr/bin/env python

import numpy as np
import teleop_utils as utils
from spatialmath import *
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
        if (button[i,1] == 1):
            anchors.append([button[i,0], button[i+1,0]])
    return anchors

def filter_poses_by_time(time_stamps, time_range, user_input_traj):
    # Find the indices where the time values in the poses array fall within the given range
    index_within_range = np.where(
        (time_stamps >= time_range[0]) & (time_stamps <= time_range[1]))
    relative_time = time_stamps[index_within_range[0]] - time_stamps[index_within_range[0][0]]
    return (user_input_traj[index_within_range[0],:,:],relative_time)
   
def rel_pose_traj(user_input_traj,command_reference_frame='moving_end_effector'):
    """
    Returns the change in the pen's pose wrt the Haptic Device base frame
    """
    start_pose = user_input_traj[0]     # Anchor pose in the haptic device base frame
    if command_reference_frame=='fixed_robot_base':
        return SE3([pose * start_pose.inv() for pose in user_input_traj])
    elif command_reference_frame=='moving_end_effector':
        return SE3([start_pose.inv()*pose for pose in user_input_traj])
    else:
        print(f'Wrong Input for command_reference_frame as {command_reference_frame}')
        return None


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
    button_enable = data[config['enable_button']]
    pose_msg = data['pose_msg']
    user_input_traj = data['user_input_traj']

    anchors = anchor_events(button_enable)
    selected_anchor_range = anchors[0]
    total_time = selected_anchor_range[1] - selected_anchor_range[0]

    (user_input_traj_fltr, rel_time) = filter_poses_by_time(
        time_stamps = pose_msg[:,0], time_range = selected_anchor_range, user_input_traj = user_input_traj)



    user_input_traj_fltr = utils.ndarray_to_se3(user_input_traj_fltr) # this forced conversion using SE3(list of SE3 instances) is necessary to enable user_input_traj.plot() 

    rel_traj = rel_pose_traj(user_input_traj=user_input_traj_fltr,
                             command_reference_frame=config['command_reference_frame'])

    # save everything
    np.savez(config['user_input_data'],
            # original data already loaded 
            button1=data['button1'], 
            button2=data['button2'], 
            pose_msg=data['pose_msg'], 
            user_input_traj=data['user_input_traj'], 
            # teleop_traj_config data
            haptic_R_viewer=config['haptic_R_viewer'],
            viewer_R_robotbase=config['viewer_R_robotbase'],
            scaling_factor=config['scaling_factor'],
            command_reference_frame=config['command_reference_frame'],
            # commanded trajectories processed,
            command_abs_traj=utils.se3_to_ndarray(user_input_traj_fltr),
            command_rel_traj=utils.se3_to_ndarray(rel_traj),
            command_time=np.array(rel_time),
            )
    
    print("File Saved As: ", config['user_input_data'])

    # Animation 1: User Input
    utils.animate_user_input(user_input_traj=user_input_traj_fltr, plt_title='User Input Traj')

    # Animation 2: User Input (Relative)
    utils.animate_user_input(user_input_traj=rel_traj, plt_title='User Input Traj (Relative)')

    # Comparison: User Input vs. Relative
    time_to_view = 1.0
    while time_to_view>0:
        time_to_view = float(input(
            f'Total time = {total_time}, select a time view (or negative time to quit): '))
        if time_to_view>=total_time:
            time_to_view = total_time
        if time_to_view > 0:
            index_to_view = int (time_to_view/total_time*len(rel_traj))
            utils.plot_user_input_pose(index = index_to_view,abs_traj=user_input_traj_fltr,rel_traj=rel_traj)

    # Comparison: Trajectory vs. Relative
    utils.plot_user_input_traj(abs_traj=user_input_traj_fltr,rel_traj=rel_traj)