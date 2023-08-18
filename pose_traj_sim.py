#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import teleop_utils as utils
import roboticstoolbox as rtb
import ipdb
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

# Load the robot model
robot = rtb.models.UR5()
haptic_to_viewer = []
viewer_to_haptic = np.array([np.array([1,0,0]),np.array([0,0,-1]),np.array([0,1,0])])

def get_desired_poses(cmd_traj, home_pose, sf):
    """
    Returns the desired robot poses from the commanded trajectory
    cmd_traj = relative trajectory input
    home_pose = starting pose of robot
    sf = scaling factor for teleoperation
    """
    des_poses = []
    for pose in cmd_traj:
        vec = np.array([home_pose.t + (viewer_to_haptic (sf * pose.t))])
        mat = pose.R @ home_pose.R
        mat = np.concatenate((mat, vec.T), axis = 1)
        mat = np.concatenate((mat, np.array([np.array([0, 0, 0, 1])])),axis = 0)
        des_poses.append(SE3(mat))

    return SE3(des_poses)

def get_velocity(position_error, position_error_norm, orientation_axis, orientation_error):
    """
    Returns the task-space velocity vector
    based on the resolved rate parameters set below
    """
    # Resolved Rate parameters
    v_min, v_max, w_min, w_max, lmda = 0.1, 1, 15, 100, 5
    # Error convergence parameters
    e_p, e_o = 0.001, 0.0524
    
    if (position_error_norm/e_p) > lmda:
        v = v_max
    else:
        v = v_min + ((v_max - v_min) * (position_error_norm - e_p) / (e_p * (lmda - 1)))

    if (orientation_error/e_o) > lmda:
        w = w_max
    else:
        w = w_min + ((w_max - w_min) * (orientation_error - e_o) / (e_o * (lmda - 1)))

    p_dot = position_error * (v / position_error_norm)
    o_dot = w * orientation_axis

    return np.concatenate((p_dot, o_dot), axis=0)

def robust_inv(joint_state):
    """
    Returns the singularity robust inverse jacobian of the robot
    {With respect to robot base frame}
    """
    jacobian = robot.jacob0(joint_state) # Jacobian wrt robot base frame
    return jacobian.T @ np.linalg.inv((jacobian @ jacobian.T) + (0.00001 * np.identity(6)))

def resolved_rate_joint_traj(traj, q_start):
    """
    Returns a trajectory of joint_state positions
    """
    # Error convergence parameters
    e_p, e_o = 0.001, 0.0524
    dt, i = 0.001, 0
    step = 0

    q_curr = q_start
    joint_state_traj = []
    joint_state_traj.append(q_curr)

    while i < len(traj):

        cur_pose = robot.fkine(q_curr)
        # Position error
        pos_err = traj[i].t - cur_pose.t
        delta_p = np.linalg.norm(pos_err)
        # Rotation error as matrix
        rot_mat = traj[i].R @ cur_pose.R.T
        rot_vec = (R.from_matrix(rot_mat)).as_rotvec()
        # Get the axis-angle of rotation error
        angle = np.linalg.norm(rot_vec)
        if angle == 0:
            axis = np.array([0,0,1])
        else:
            axis = rot_vec / angle

        while (delta_p > e_p) or (angle > e_o):
            vel = get_velocity(pos_err, delta_p, axis, angle)
            
            # Multiply with jacobian inverse for joint speeds
            q_dot = robust_inv(q_curr) @ vel

            # Get next joint states using time step
            q_curr = q_curr + (q_dot * dt)

            # Taking every 0.025 seconds of joint_state information for plotting later
            if step % 25 == 0:
                joint_state_traj.append(q_curr)
            
            step += 1

            cur_pose = robot.fkine(q_curr)
            # Position error
            pos_err = traj[i].t - cur_pose.t
            delta_p = np.linalg.norm(pos_err)
            # Rotation error as matrix
            rot_mat = traj[i].R @ cur_pose.R.T
            rot_vec = (R.from_matrix(rot_mat)).as_rotvec()
            # Get the axis-angle of rotation error
            angle = np.linalg.norm(rot_vec)
            if angle == 0:
                axis = np.array([0,0,1])
            else:
                axis = rot_vec / angle
        i += 1
    return np.array(joint_state_traj)

if __name__ == '__main__':
    # Load the command data { waypoints, timestamps }
    config = utils.load_config()
    data = np.load(config['user_input_data'])

    cmd_traj = utils.ndarray_to_se3(data['command_rel_traj'])   # This is an array of SE(3) type poses {waypoints}
    cmd_time = data['command_time'] # {timestamps}
    q_home = config['UR5_home']

    # Set the start pose as desired. Can use qr,qn,q1,etc
    T_home = robot.fkine(q_home)

    robot_traj = get_desired_poses(cmd_traj=cmd_traj, home_pose=T_home, sf=config['scaling_factor'])
    joint_traj = resolved_rate_joint_traj(robot_traj, q_home)

    gif_path = 'data_saved/UR5_' + config['name'] + '.gif'
    # The below method generates an animation
    robot.plot(joint_traj, dt=0.025, block=False, backend='pyplot', movie=gif_path)      # by default, dt=0.05

    # save everything
    np.savez(config['user_input_data'],
            # original data already loaded 
            button1=data['button1'], 
            button2=data['button2'], 
            pose_msg=data['pose_msg'], 
            user_input_traj=data['user_input_traj'], 
            command_abs_traj=data['command_abs_traj'],
            command_rel_traj=data['command_rel_traj'],
            command_time=data['command_time'],
            # commanded trajectories processed
            robot_joint_traj=joint_traj,
            )
    print("File Saved As: ", config['user_input_data'])