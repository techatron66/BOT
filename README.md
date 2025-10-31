# BOT
ROS MELODIC BASED JESTSON NANO AUTONOMOUS ROBOT

# Build the Package
cd ~/catkin_ws

catkin_make

source devel/setup.bash


Testing - Display Robot in RViz Only
To view the robot model in RViz without Gazebo:
bashroslaunch articubot_one display.launch
This will open RViz with your robot model. You can use the joint_state_publisher GUI to manually move the wheels.

Testing - Gazebo Simulation
To launch the robot in Gazebo:

roslaunch articubot_one gazebo.launch

This will:

Start Gazebo with your robot spawned
Start robot_state_publisher for TF transforms
Enable the differential drive controller

Teleop Control
In a new terminal, run teleop_twist_keyboard:
bash# Install if not already installed

sudo apt-get install ros-melodic-teleop-twist-keyboard

# Run teleop

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Controls:

i - Forward
, - Backward
j - Turn Left
l - Turn Right
k - Stop
q/z - Increase/decrease max speeds
w/x - Increase/decrease only linear speed
e/c - Increase/decrease only angular speed

View in RViz with Gazebo Running
While Gazebo is running, open RViz in a new terminal:

rosrun rviz rviz

Then:

Set Fixed Frame to odom
Add RobotModel display
Add TF display
Optionally add Odometry display (topic: /odom)