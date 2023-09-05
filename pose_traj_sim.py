#!/usr/bin/env python

import numpy as np
import teleop_utils as utils
import roboticstoolbox as rtb
import ipdb
from spatialmath import SE3, SO3
from scipy.spatial.transform import Rotation as R
import yaml

# LOAD THE ROBOT MODEL
ROBOT = rtb.models.UR5()

# RESOLVED RATE PARAMETERS
V_MIN, V_MAX, W_MIN, W_MAX, LMDA, DT = 0.1, 1, 15, 100, 5, 0.001
# ERROR CONVERGENCE PARAMETERS
E_P, E_O = 0.001, 0.0524

def get_desired_poses(cmd_traj, home_pose, cmd, sf, haptic_R_viewer, viewer_R_robotbase, command_reference_frame='fixed_robot_base'):
    """
    Returns the desired robot poses from the commanded trajectory as an ndarray

    Parameters:
    cmd_traj = ndarray type relative trajectory input wrt Haptic Base frame
    home_pose = starting pose of robot wrt Robot Base frame
    sf = scaling factor for teleoperation
    haptic_R_viewer = the rotation matrix that rewrites a vector written in the viewer base frame to one written in the haptic base frame
    viewer_R_robotbase = the rotation matrix that rewrites a vector written in the robot base frame to one written in the viewer frame
    """
    # The haptic frame with reference to the robot base frame
    anch = utils.ndarray_to_se3(cmd)[0]
    haptic_R_anch = SE3(SO3(anch.R))
    robotbase_R_haptic = viewer_R_robotbase.inv() * haptic_R_viewer.inv()
    # scaling applied to cartesian positions
    cmd_traj[:,0:3,3] *= sf
    print("Working in " + command_reference_frame + " convention")
    if command_reference_frame=='fixed_robot_base':
        des_poses = [((robotbase_R_haptic * rel_pose * robotbase_R_haptic.inv()) * home_pose) for rel_pose in utils.ndarray_to_se3(cmd_traj)]
    elif command_reference_frame=='moving_end_effector':
        des_poses = [(home_pose * (robotbase_R_haptic * rel_pose * robotbase_R_haptic.inv())) for rel_pose in utils.ndarray_to_se3(cmd_traj)]
    else:
        print(f'Wrong Input for command_reference_frame as {command_reference_frame}')
        return None
    return utils.se3_to_ndarray(SE3(des_poses))

def get_velocity(position_error, position_error_norm, orientation_axis, orientation_error):
    """
    Returns the task-space velocity vector
    based on the resolved rate parameters set below
    """
    if (position_error_norm/E_P) > LMDA:
        v = V_MAX
    else:
        v = V_MIN + ((V_MAX - V_MIN) * (position_error_norm - E_P) / (E_P * (LMDA - 1)))

    if (orientation_error/E_O) > LMDA:
        w = W_MAX
    else:
        w = W_MIN + ((W_MAX - W_MIN) * (orientation_error - E_O) / (E_O * (LMDA - 1)))

    p_dot = position_error * (v / position_error_norm)
    o_dot = w * orientation_axis

    return np.concatenate((p_dot, o_dot), axis=0)

def robust_inv(joint_state):
    """
    Returns the singularity robust inverse jacobian of the robot
    {With respect to robot base frame}
    """
    jacobian = ROBOT.jacob0(joint_state) # Jacobian wrt robot base frame
    return jacobian.T @ np.linalg.inv((jacobian @ jacobian.T) + (0.00001 * np.identity(6)))

def resolved_rate_joint_traj(traj, q_start):
    """
    Returns a trajectory of joint_state positions
    """
    i = 0
    step = 0
    traj = utils.ndarray_to_se3(traj)
    q_curr = q_start
    joint_state_traj = []
    joint_state_traj.append(q_curr)

    while i < len(traj):
        cur_pose = ROBOT.fkine(q_curr)
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

        while (delta_p > E_P) or (angle > E_O):
            vel = get_velocity(pos_err, delta_p, axis, angle)
            
            # Multiply with jacobian inverse for joint speeds
            q_dot = robust_inv(q_curr) @ vel

            # Get next joint states using time step
            q_curr = q_curr + (q_dot * DT)

            # Taking every 0.025 seconds of joint_state information for plotting later
            if step % 25 == 0:
                joint_state_traj.append(q_curr)
            
            step += 1

            cur_pose = ROBOT.fkine(q_curr)
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

def create_yaml(data_to_convert, file_name):
    # Convert the parent ndarray and nested ndarrays to the desired format for YAML
    yaml_data = []
    for idx, sublist in enumerate(data_to_convert):
        yaml_data.append({ 
            'Joint_Positions': sublist.tolist(),
            'traj_point': idx,
        })

    # Write the data to the YAML file
    with open(file_name, 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file, default_flow_style=False)
    print('joint state trajectory saved as: ', file_name)

if __name__ == '__main__':
    # Load the command data { waypoints, timestamps }
    config = utils.load_config()
    haptic_R_viewer = SE3(config['haptic_R_viewer'])
    viewer_R_robotbase = SE3(config['viewer_R_robotbase'])

    data = np.load(config['user_input_data'])
    # Relative pen poses wrt the Haptic Device base frame
    cmd_traj = data['command_rel_traj']
    cmd_time = data['command_time'] # {timestamps}

    q_home = config['follower_robot_home']
    T_home = ROBOT.fkine(q_home)

    robot_traj = get_desired_poses( 
                                    cmd_traj=cmd_traj, 
                                    home_pose=T_home,
                                    cmd=data['command_abs_traj'],
                                    sf=config['scaling_factor'], 
                                    haptic_R_viewer=config['haptic_R_viewer'], 
                                    viewer_R_robotbase=config['viewer_R_robotbase'],
                                    command_reference_frame=config['command_reference_frame']
                                    )
    
    joint_traj = resolved_rate_joint_traj(robot_traj, q_home)

    gif_path = 'data_saved/follower_robot_' + config['name'] + '.gif'
    
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
            # teleop_traj_config data
            haptic_R_viewer=config['haptic_R_viewer'],
            viewer_R_robotbase=config['viewer_R_robotbase'],
            scaling_factor=config['scaling_factor'],
            command_reference_frame=config['command_reference_frame'],
            # robot trajectory processed
            robot_pose_traj=robot_traj,
            robot_joint_traj=joint_traj,
            )
    print("File Saved As: ", config['user_input_data'])
    # Creates yaml configuration file to use with ROS node
    create_yaml(joint_traj, config['yaml_file_path'])

    input("Display the input vs robot trajectory comparison: press [Enter]")
    # Map both the input and robot trajectory to the viewer frame
    hap_traj = haptic_R_viewer.inv() * utils.ndarray_to_se3(data['command_abs_traj'])
    rob_traj = viewer_R_robotbase * utils.ndarray_to_se3(robot_traj)
    # plot the comparison of both
    utils.plot_haptic_robot_traj(hap_traj=hap_traj,rob_traj=rob_traj)
    
    # The below method generates an animation
    input("Press [enter] to display animated trajectory")
    ROBOT.plot(joint_traj, dt=0.025, block=False, backend='pyplot', movie=gif_path)      # by default, dt=0.05