# ROS Melodic Based Jetson Nano Autonomous Robot

A differential drive robot project using ROS Melodic on Jetson Nano. Supports both **Gazebo simulation** and **real hardware deployment** with Arduino-based motor control.

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Package Structure](#package-structure)
- [Setup Instructions](#setup-instructions)
- [Simulation](#simulation)
- [Real Hardware (Arduino + L293D)](#real-hardware-arduino--l293d)
- [Teleop Control](#teleop-control)
- [SLAM and Navigation](#slam-and-navigation)
- [Camera](#camera)
- [Useful Commands](#useful-commands)
- [Troubleshooting](#troubleshooting)

---

## Features

- Gazebo simulation with LIDAR and Camera
- RViz visualization
- Keyboard teleoperation
- **Real hardware control via Arduino + L293D motor driver**
- **ROSSerial bridge for ROS-Arduino communication**
- SLAM (GMapping)
- Autonomous navigation (AMCL + move_base)
- Autonomous exploration (explore_lite)
- RPLidar integration

---

## Hardware Requirements

### For Simulation Only
- Ubuntu 18.04 / Jetson Nano
- ROS Melodic

### For Real Robot (Minimum Setup - No Encoders)
- Jetson Nano (or any Ubuntu 18.04 machine with ROS Melodic)
- Arduino Uno/Nano
- L293D Motor Driver (or L298N)
- 2x DC Motors
- USB cable (Arduino to Jetson)
- External power supply for motors (battery pack)

### For Full Autonomous Mode
- All items above, plus:
- Wheel encoders
- RPLidar A1/A2
- Motor driver with higher current capacity (if needed)

---

## Package Structure

```
articubot_one/
├── CMakeLists.txt
├── package.xml
├── arduino/
│   └── arduino_motor_bridge.ino    # Arduino sketch for motor control
├── urdf/
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   ├── inertial_macros.xacro
│   ├── gazebo_control.xacro
│   └── lidar.xacro
├── launch/
│   ├── gazebo.launch
│   ├── gazebo_with_lidar.launch
│   ├── gazebo_with_camera.launch
│   ├── display.launch
│   ├── hardware_bringup.launch     # Real robot bringup (no encoders)
│   ├── teleop_hardware.launch      # Teleop + hardware bringup
│   ├── mapping_teleop.launch
│   ├── navigation.launch
│   ├── explore_lite.launch
│   └── rplidar.launch
└── rviz/
    ├── robot_view.rviz
    └── robot_with_lidar.rviz
```

---

## Setup Instructions

### 1. Create the Package

```bash
cd ~/catkin_ws/src
catkin_create_pkg articubot_one rospy roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf urdf xacro gazebo_ros gazebo_plugins robot_state_publisher joint_state_publisher
```

### 2. Create Directories

```bash
cd ~/catkin_ws/src/articubot_one
mkdir -p arduino urdf launch rviz
```

### 3. Add Files

- Place all `.xacro` files in `urdf/`
- Place all `.launch` files in `launch/`
- Place `.rviz` files in `rviz/`
- Place Arduino sketch in `arduino/`

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
install(DIRECTORY arduino/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/arduino)
```

### 5. Install Dependencies

```bash
# Teleop
sudo apt-get install ros-melodic-teleop-twist-keyboard

# For real hardware bridge
sudo apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial-python

# For SLAM and Navigation
sudo apt-get install -y ros-melodic-gmapping ros-melodic-navigation ros-melodic-map-server ros-melodic-amcl

# For autonomous exploration
sudo apt-get install -y ros-melodic-explore-lite

# For LIDAR (real robot)
sudo apt-get install ros-melodic-rplidar-ros
```

### 6. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Simulation

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

---

## Real Hardware (Arduino + L293D)

This setup allows you to control a real differential drive robot using keyboard teleop, with Arduino handling motor control via the L293D driver.

### Wiring Diagram

```
Jetson Nano USB  ----USB Cable---->  Arduino USB
                                     |
                                     |---> D3 (ENA)  ---> L293D ENA (Motor A PWM)
                                     |---> D5 (IN1)  ---> L293D IN1
                                     |---> D6 (IN2)  ---> L293D IN2
                                     |---> D9 (ENB)  ---> L293D ENB (Motor B PWM)
                                     |---> D7 (IN3)  ---> L293D IN3
                                     |---> D8 (IN4)  ---> L293D IN4
                                     |
L293D Power:
  +5V, GND    ---> Arduino 5V/GND
  +12V (VCC2) ---> External motor battery (+)
  GND         ---> Common ground (Arduino GND + Battery GND)

L293D Outputs:
  OUT1, OUT2  ---> Motor A (Left Wheel)
  OUT3, OUT4  ---> Motor B (Right Wheel)
```

### Arduino Setup

1. Install the Arduino ROS library:
   - Open Arduino IDE
   - Sketch -> Include Library -> Manage Libraries
   - Search for "Rosserial Arduino Library" and install it

2. Open `arduino/arduino_motor_bridge.ino` in Arduino IDE

3. Upload the sketch to your Arduino board

4. Verify the Arduino is detected:
   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB*
   # Usually /dev/ttyACM0 for Arduino Uno
   ```

5. Set permissions (or add user to `dialout` group):
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

### Launch Real Robot with Teleop

**Single command launch (brings up everything):**

```bash
roslaunch articubot_one teleop_hardware.launch
```

This launch file will:
1. Start the Arduino serial node (rosserial)
2. Load robot URDF and publish TF frames
3. Start joint_state_publisher (for RViz visualization)
4. Publish static odom->base_link transform (placeholder until encoders are added)
5. Open RViz with robot model
6. Start keyboard teleop node

### Manual Component Launch (for debugging)

**Terminal 1 - Arduino bridge:**
```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

**Terminal 2 - Robot description & TF:**
```bash
roslaunch articubot_one hardware_bringup.launch
```

**Terminal 3 - Teleop:**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Test Motors Without Teleop

```bash
# Publish a simple forward command
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

---

## Teleop Control

### Install

```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```

### Controls

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn Left |
| `l` | Turn Right |
| `k` | Stop |
| `q/z` | Increase/decrease max speeds by 10% |
| `w/x` | Increase/decrease only linear speed by 10% |
| `e/c` | Increase/decrease only angular speed by 10% |

### Adjusting Speed for Real Hardware

Edit `teleop_hardware.launch` to change default speeds:

```xml
<param name="speed" value="0.3"/>    <!-- Linear speed m/s -->
<param name="turn" value="0.5"/>     <!-- Angular speed rad/s -->
```

**Note:** For the initial test, start with low speeds (0.1-0.2 m/s) to verify motor direction and wiring.

---

## Camera

### Simulation

The camera publishes to `/camera/image_raw` at 30 Hz with 640x480 resolution.

```bash
rosrun image_view image_view image:=/camera/image_raw
```

### Real Hardware (Optional)

If you add a USB camera to the Jetson:

```bash
sudo apt-get install ros-melodic-usb-cam
rosrun usb_cam usb_cam_node
```

---

## SLAM and Navigation

### Prerequisites

```bash
sudo apt-get install -y ros-melodic-gmapping ros-melodic-navigation ros-melodic-map-server ros-melodic-amcl
```

### 1. Mapping (with real robot + teleop)

**Terminal 1 - Robot bringup + mapping:**
```bash
roslaunch articubot_one mapping_teleop.launch
```

**Terminal 2 - Teleop control:**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Drive the robot slowly to build the map.

### 2. Save the Map

```bash
mkdir -p ~/catkin_ws/src/articubot_one/maps
cd ~/catkin_ws/src/articubot_one/maps
rosrun map_server map_saver -f my_map
```

This produces `my_map.yaml` and `my_map.pgm`.

### 3. Navigate with Saved Map

```bash
roslaunch articubot_one navigation.launch map:=/home/<user>/catkin_ws/src/articubot_one/maps/my_map.yaml
```

In RViz:
- Click **"2D Pose Estimate"** to set the robot's initial pose
- Click **"2D Nav Goal"** to send navigation goals

### Autonomous Exploration

```bash
sudo apt-get install -y ros-melodic-explore-lite
roslaunch articubot_one explore_lite.launch
```

---

## Useful Commands

### Diagnostics

```bash
rostopic list                    # Check active topics
rostopic echo /cmd_vel           # Monitor velocity commands
rostopic echo /scan              # Monitor LIDAR data
rostopic hz /scan                # Check LIDAR scan rate
rostopic echo /odom              # Monitor odometry
rosrun tf view_frames            # Check TF tree
rqt_graph                        # View node/topic graph
```

### Arduino Debugging

```bash
# Check if Arduino serial node is connected
rostopic echo /cmd_vel

# Test motor response directly
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

---

## Troubleshooting

### Simulation Issues

#### Robot falls through ground
- Check inertial properties in URDF
- Verify collision geometries

#### Robot doesn't move in Gazebo
- Check `/cmd_vel` topic presence
- Verify controller loading in `gazebo_control.xacro`

#### TF errors in RViz
- Confirm `robot_state_publisher` is running
- Check Fixed Frame setting (should be `base_link` or `odom`)

### Real Hardware Issues

#### Arduino not detected
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
# Try different USB ports
# May appear as /dev/ttyUSB0 instead of /dev/ttyACM0
```

#### Permission denied on serial port
```bash
# Quick fix
sudo chmod 666 /dev/ttyACM0

# Permanent fix - add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

#### Motors not spinning
- Verify L293D has separate motor power supply (VCC2)
- Check common ground between Arduino and motor power supply
- Test with Arduino IDE Serial Monitor first
- Verify `ENA` and `ENB` are connected to PWM pins (3, 9)

#### Motors spin in wrong direction
- Swap IN1/IN2 wires for Motor A
- Swap IN3/IN4 wires for Motor B

#### Motors spin at different speeds
- Adjust PWM scaling factor in Arduino code
- Check battery voltage (low battery causes uneven performance)

#### RViz shows robot but no wheel movement
- Normal for no-encoder setup - wheels won't animate in RViz
- The static transform keeps the robot visible
- Wheel animation requires encoder feedback (future enhancement)

#### Rosserial connection drops
- Check baud rate matches (57600 in provided code)
- Ensure Arduino is not resetting due to USB power issues
- Try adding a delay at the start of `setup()`

---

## Next Steps / Roadmap

### Phase 1: Basic Movement (Current)
- [x] Arduino motor bridge with L293D
- [x] Keyboard teleop control
- [x] RViz visualization
- [x] ROS-Arduino serial bridge

### Phase 2: Add Encoders
- [ ] Install wheel encoders
- [ ] Update Arduino code to read encoder ticks
- [ ] Publish `nav_msgs/Odometry` from Arduino
- [ ] Remove static `odom->base_link` transform
- [ ] Add proper odometry calculation

### Phase 3: Add LIDAR
- [ ] Install RPLidar
- [ ] Verify `/scan` topic
- [ ] Test SLAM with real robot
- [ ] Autonomous navigation

### Phase 4: Advanced
- [ ] IMU integration (MPU6050/9250)
- [ ] EKF sensor fusion (robot_localization)
- [ ] Camera-based navigation
- [ ] Autonomous exploration

---

## License

This project is open source. Feel free to modify and distribute.

## Credits

Built for ROS Melodic on Jetson Nano. Arduino motor bridge adapted for L293D dual H-bridge driver.
