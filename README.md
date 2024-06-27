# EKF localization Pammer
This is an Extended Kalman Filter (EKF) localization implementation with a Turtlebot3 in ROS (Robot Operating System).

## Launch
**To run the application, execute the start.launch launchfile** in the pammer package. This will launch: 
- Turtlebot3 in Gazebo
- Rviz (with preset configuration)
- ekf_node
- goals_node

**& roslaunch pammer start.launch**

## Basic functionality
### goals_node
This node moves the Turlebot3 via the Navigation Stack sequentially to four predetermined poses. The goals are defined in "goals_param.yaml".

### ekf_node
This node is the implementation of the EKF localization. Via a subscriber and ROS message filters, synchronized odometry and LiDAR data is obtained from the Turtlebot3. With the received odometry data, a pose estimation/prediction is carried out. Landmark detection is implemented with LiDAR data and the 2D circle RANSAC method from PCL (Point Cloud Library). The singular landmark (a circular cone) - used for the correction step - has predefined coordinates in the world frame. When a circle is detected, that at most deviates 1 meter from the landmarks coordinates, the correction step is called.

## Requirements:
PCL - download here: https://pointclouds.org/downloads/
