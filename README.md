# BOT — ArticuBot One (Jetson Nano, ROS Melodic)

Lightweight guide to build, run and visualize the ArticuBot One robot (ROS Melodic). Includes instructions for displaying the URDF model in RViz, running a Gazebo simulation, and teleoperation.

## Features
- Visualize robot model in RViz
- Run full Gazebo simulation with controllers
- Teleoperate using keyboard commands

## Prerequisites
- Ubuntu + ROS Melodic installed
- catkin workspace at ~/catkin_ws
- (Optional) Gazebo and rviz installed
- On Jetson Nano: follow NVIDIA Jetson setup for ROS Melodic if needed

## Build
Open a terminal and run:
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Quick: Display robot in RViz (no Gazebo)
To view only the robot model in RViz:
```bash
roslaunch articubot_one display.launch
```
- RViz will open with the robot model.
- Use the joint_state_publisher GUI to move joints manually.

## Full Simulation in Gazebo
Launch Gazebo with the robot and controllers:
```bash
roslaunch articubot_one gazebo.launch
```
This will:
- Start Gazebo and spawn the robot
- Start robot_state_publisher (TF transforms)
- Enable the differential drive controller

## Teleoperation (keyboard)
Install teleop package (if needed) and run:
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Controls:
- i — Forward
- , — Backward
- j — Turn left
- l — Turn right
- k — Stop
- q / z — Increase / decrease max speeds
- w / x — Increase / decrease linear speed only
- e / c — Increase / decrease angular speed only

## View in RViz while Gazebo is running
Open RViz:
```bash
rosrun rviz rviz
```
Recommended displays:
- Fixed Frame: odom
- RobotModel
- TF
- (Optional) Odometry — topic: /odom

## Troubleshooting
- If RViz shows no model: ensure your URDF is published and robot_state_publisher is running.
- If controllers don't start: check controller_manager and rostopic list for expected topics.
- If permissions or network issues occur when connecting to a remote Gazebo/ROS master, verify ROS_MASTER_URI and ROS_HOSTNAME.

## License
Include your preferred license here (e.g., MIT, Apache-2.0).