Explanation of Fields in a Trajectory configuration file

## [General]
* ***user_input_rosbag***: the filename of a rosbag file for the raw user input data colection of a haptic device
* ***scaling_factor***: a gain to use on the linear motion command during teleoperation
* ***haptic_R_viewer***: the rotation matrix that rewrites a vector written in the viewer base frame to one written in the haptic base frame
* ***viewer_R_robotbase***:  the rotation matrix that rewrites a vector written in the robot base frame to one written in the viewer frame
* ***command_reference_frame***: one key step in teleoperation is that relative poses will be captured between an anchor pose and a moving pose after the anchor. However, there are two ways of capturing this relative: either in a fixed frame convention, or in a moving frame convention.  
    * Option [***fixed_robot_base***]: the relative command will be captured using fixed frame convention, and enventually used in the robot_base frame
    * Option [***moving_end_effector***]: the relative command will be captured using moving frame convention, and enventually used in the end_effector frame

## [Robot]
* ***joint_states_home***: joint positions at home