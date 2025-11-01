# ROS 1 Melodic Robot Setup Guide

This guide explains how to set up a differential drive robot using ROS Melodic.

## Package Structure

Create your ROS package with the following structure:
```
articubot_one/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   ├── inertial_macros.xacro
│   ├── gazebo_control.xacro
│   └── lidar.xacro
├── launch/
│   ├── gazebo.launch
│   ├── gazebo_with_lidar.launch
│   ├── display.launch
│   └── rplidar.launch
└── rviz/
    ├── robot_view.rviz
    └── robot_with_lidar.rviz
```

## Setup Instructions

### 1. Create the Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg articubot_one rospy roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf urdf xacro gazebo_ros gazebo_plugins robot_state_publisher joint_state_publisher
```

### 2. Create Directories
```bash
cd ~/catkin_ws/src/articubot_one
mkdir -p urdf launch rviz
```

### 3. Add Files
- Place all `.xacro` files in `urdf/`
- Place all `.launch` files in `launch/`
- Place `.rviz` files in `rviz/`

### 4. Update CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(articubot_one)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp std_msgs sensor_msgs geometry_msgs nav_msgs
  tf urdf xacro gazebo_ros robot_state_publisher joint_state_publisher
)

catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
```

### 5. Build the Package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### View Robot in RViz
```bash
roslaunch articubot_one display.launch
```

### Launch Gazebo Simulation with LIDAR
```bash
roslaunch articubot_one gazebo_with_lidar.launch
```

### Launch Gazebo Simulation with LIDAR and Camera
```bash
roslaunch articubot_one gazebo_with_camera.launch
```

### Teleop Control
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### Teleop Controls
- `i` - Forward
- `,` - Backward
- `j` - Turn Left
- `l` - Turn Right
- `k` - Stop
- `q/z` - Increase/decrease speeds
- `w/x` - Adjust linear speed
- `e/c` - Adjust angular speed

### Camera View
The camera publishes to `/camera/image_raw` at 30 Hz with 640x480 resolution. View the camera feed with:
```bash
rosrun image_view image_view image:=/camera/image_raw
```

## Useful Commands

### Diagnostics
```bash
rostopic list           # Check active topics
rostopic echo /scan    # Monitor LIDAR data
rostopic hz /scan      # Check LIDAR scan rate
rostopic echo /odom    # Monitor odometry
rostopic echo /cmd_vel # Monitor velocity commands
rosrun tf view_frames  # Check TF tree
```

## Troubleshooting

### Robot falls through ground
- Check inertial properties
- Verify collision geometries

### Robot doesn't move
- Check `/cmd_vel` topic presence
- Verify controller loading

### TF errors in RViz
- Confirm robot_state_publisher is running
- Check Fixed Frame setting

## Real Robot Integration

### Required Hardware
- Arduino board
- Motor driver
- Wheel encoders
- RPLidar sensor
- Jetson Nano (recommended)

### Software Setup
1. Install ROS Melodic on Jetson Nano
2. Configure rosserial_arduino
3. Install RPLidar package:
```bash
sudo apt-get install ros-melodic-rplidar-ros
```

### LIDAR Setup
1. Connect RPLidar to USB
2. Identify port: `ls -l /dev/ttyUSB*`
3. Update `rplidar.launch` with correct port
4. Test: `roslaunch articubot_one rplidar.launch`

