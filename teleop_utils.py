#!/usr/bin/env python

import os
import sys
import configparser
import numpy as np
from spatialmath import *
import matplotlib.pyplot as plt
import ast
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
    yaml_file_path = os.path.join('data_saved',config_file_name+'.yaml')
    
    sf = ast.literal_eval(config['General']['scaling_factor'])

    # get the viewer perspective frames 
    haptic_R_viewer = ast.literal_eval(config['General']['haptic_R_viewer'])
    try:    # Returns identity SE(3) if the user input rotation is invalid
        haptic_R_viewer = SE3(SO3(np.array([np.array(i) for i in haptic_R_viewer])))
    except:
        haptic_R_viewer = SE3(SO3())

    viewer_R_robotbase = ast.literal_eval(config['General']['viewer_R_robotbase'])
    try:    # Returns identity SE(3) if the user input rotation is invalid
        viewer_R_robotbase = SE3(SO3(np.array([np.array(i) for i in viewer_R_robotbase])))
    except:
        viewer_R_robotbase = SE3(SO3())

    # Get the robot home joint state
    q_home = np.radians(np.array(ast.literal_eval(config['Robot']['joint_states_home'])))

    # output
    config_data = {
        'name':config_file_name,
        'user_input_rosbag': rosbag_file_path,
        'user_input_data': user_input_data_path,
        'enable_button' : config['General']['enable_button'],
        'teleop_command_data': teleop_command_data_path,
        'haptic_R_viewer': haptic_R_viewer,
        'viewer_R_robotbase': viewer_R_robotbase,        
        'scaling_factor': sf,
        'follower_robot_home': q_home,
        'yaml_file_path': yaml_file_path,
        'command_reference_frame': config['General']['command_reference_frame']
    }
    return config_data

def ndarray_to_se3(nd_array: np.ndarray):
    return SE3([SE3(nd_array[i,:,:]) for i in range(nd_array.shape[0])])

def se3_to_ndarray(se3_array: SE3):
    return np.array([np.array(x) for x in se3_array])

def animate_user_input(user_input_traj:SE3,plt_title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.title(plt_title)
    user_input_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    user_input_traj.animate(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.show(block=False)
    input(f'Animation [{plt_title}] displayed')

def plot_user_input_pose(index,abs_traj,rel_traj):
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    abs_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    abs_traj[index].plot(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj')
    
    ax = fig.add_subplot(122, projection='3d')
    rel_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    rel_traj[index].plot(frame='i', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj (Relative)')

    plt.show(block=False)
    input(f'User Input Pose [{index}] displayed')

def plot_user_input_traj(abs_traj,rel_traj, skip_N=75):
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    abs_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    abs_traj[-1].plot(frame='end', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj')
    abs_traj_nd = se3_to_ndarray(abs_traj)
    ax.plot(
        abs_traj_nd[::skip_N,0,3],
        abs_traj_nd[::skip_N,1,3],
        abs_traj_nd[::skip_N,2,3],
        label='trajectory curve'
        )

    ax = fig.add_subplot(122, projection='3d')
    rel_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    rel_traj[-1].plot(frame='end', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input Traj (Relative)')
    rel_traj_nd = se3_to_ndarray(rel_traj)
    ax.plot(
        rel_traj_nd[::skip_N,0,3],
        rel_traj_nd[::skip_N,1,3],
        rel_traj_nd[::skip_N,2,3],
        label='trajectory curve'
        )
    plt.show(block=False)
    input(f'User Input Traj displayed')

def plot_haptic_robot_traj(hap_traj,rob_traj, skip_N=75):
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    hap_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    hap_traj[-1].plot(frame='end', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('User Input (Haptic) Traj')
    hap_traj_nd = se3_to_ndarray(hap_traj)
    ax.plot(
        hap_traj_nd[::skip_N,0,3],
        hap_traj_nd[::skip_N,1,3],
        hap_traj_nd[::skip_N,2,3],
        label='trajectory curve'
        )

    ax = fig.add_subplot(122, projection='3d')
    rob_traj[0].plot(frame='st', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    rob_traj[-1].plot(frame='end', style='rgb', axislabel=False, originsize=50, length=0.1, flo=(-0.01,-0.01,-0.01))
    plt.title('UR5 TCP Traj')
    rob_traj_nd = se3_to_ndarray(rob_traj)
    ax.plot(
        rob_traj_nd[::skip_N,0,3],
        rob_traj_nd[::skip_N,1,3],
        rob_traj_nd[::skip_N,2,3],
        label='trajectory curve'
        )
    plt.show(block=False)
    input(f'Trajectory Comparison displayed')
