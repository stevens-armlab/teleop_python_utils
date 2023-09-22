#!/usr/bin/env python

import numpy as np
import teleop_utils as utils
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import ipdb

# LOAD THE ROBOT MODEL
ROBOT = rtb.models.UR5()

def get_metrics(joint_poses):
    """
    Returns the manipulability measure
    """
    mu = []
    inv_cond_num = []

    for pose in joint_poses:
        # Do SVD
        _, S, _ = np.linalg.svd(ROBOT.jacob0(pose), True)
        
        # Get Manipulability Measure
        mu.append(np.prod(S))

        # Get Inverse Condition Number
        inv_cond_num.append(np.min(S)/np.max(S))

    return mu, inv_cond_num

def manip_ell(joint_poses, robot_twist):
    """
    WORK IN PROGRESS
    This function aims to plot the manipulability ellipsoids of the robot end effector
    """
    for pose, twist in zip(joint_poses, robot_twist):
        # Do SVD
        U, S, Vh = np.linalg.svd(ROBOT.jacob0(pose), True)
        twist_tilda = U.T @ twist

    # Define ellipsoid parameters for multiple ellipsoids
    centers = np.array([[1, 2, 3], [-2, 0, 1], [0, -2, 2]])  # Center coordinates for each ellipsoid
    radii = np.array([[2, 1, 0.5], [1.5, 1, 1], [1, 2, 2]])  # Radii for each ellipsoid
    colors = ['b', 'g', 'r']  # Colors for each ellipsoid

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Loop through each ellipsoid and plot them
    for i in range(len(centers)):
        center = centers[i]
        radius = radii[i]
        color = colors[i]

        # Create a meshgrid of phi and theta values
        phi, theta = np.meshgrid(np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 50))

        # Parametric equations for the ellipsoid
        x = center[0] + radius[0] * np.sin(theta) * np.cos(phi)
        y = center[1] + radius[1] * np.sin(theta) * np.sin(phi)
        z = center[2] + radius[2] * np.cos(theta)

        # Plot the ellipsoid
        ax.plot_surface(x, y, z, color=color, alpha=0.7)

    # Set axis labels (optional)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()


if __name__ == '__main__':
    config = utils.load_config()
    data = np.load(config['user_input_data'])

    robot_joint_traj, robot_twist_traj = data['robot_joint_traj'], data['robot_twist_traj']

    mu, inv_cond_num = get_metrics(robot_joint_traj)

    # utils.parametrized_plot(mu, "$\mu$", "Manipulability Measure $\mu$")
    utils.parametrized_plot(inv_cond_num, "$\kappa$", "Inverse Condition Number $\kappa$")
