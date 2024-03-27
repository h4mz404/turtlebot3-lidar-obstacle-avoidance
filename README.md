Course: ENPM690 Spring 2024
Assignment: HW3
Author : Hamza Shah Khan
UID: 119483152
Directory ID: hamzask@umd.edu
---

# RoboMove ROS2 Package

This ROS2 package, **robomove_pkg**, provides functionality for obstacle avoidance using a Lidar sensor mounted on a TurtleBot3 robot in a Gazebo simulation environment. It leverages various ROS2 packages, including TurtleBot3 packages, Dynamixel SDK, and custom interfaces.

## Prerequisites

Ensure that you have the following ROS2 packages installed:

- `turtlebot3_msgs`
- `dynamixel_sdk`
- `hls_lfcd_lds_driver`
- `turtlebot3_description`
- `dynamixel_sdk_custom_interfaces`
- `turtlebot3_cartographer`
- `turtlebot3_gazebo`
- `turtlebot3_navigation2`
- `turtlebot3_teleop`
- `ld08_driver`
- `robomove_pkg`


## Notes

- **Important:** Before running the commands, ensure that:
  1. Place the unzipped `robomove_pkg` in a ROS2 workspace with `turtlebot3` installed.
  2. Put the contents of `robomove_pkg/launch` and `robomove_pkg/worlds` into the respective folders of the `turtlebot3_gazebo` package.
  3. Add these lines to your ~/.bashrc :
  
  export TURTLEBOT3_MODEL=waffle
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/

## Build

Build the required packages using colcon:

  $colcon build

## Source

Source your ROS2 workspace setup file:

   
   $source /path/to/your/ros2_workspace/install/setup.bash

   Replace `/path/to/your/ros2_workspace` with the actual path to your ROS2 workspace.
## Run Gazebo Simulation

1. Launch the Gazebo simulation with the TurtleBot3 model:


   $ros2 launch turtlebot3_gazebo enpm690_model.launch.py

## Part 1: Run Teleop Node

2. In another terminal window, run the teleop node:


   $ros2 run robomove_pkg teleop_keyboard

   Output Video: https://drive.google.com/file/d/1auAgp_om0UlxpD5pA-yRnVTRNTa90Ht0/view?usp=sharing


## Part 2: Run Obstacle Avoidance Node

2. In another terminal window, run the obstacle avoidance node:


   $ros2 run robomove_pkg lidar_obstacle_avoidance

   Output Video: https://drive.google.com/file/d/1WAB5d2H5oDbhWNq63-u8PoGx9SCoflkp/view?usp=drive_link





