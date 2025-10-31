cmake_minimum_required(VERSION 3.0.2)
project(articubot_one)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  urdf
  xacro
  gazebo_ros
  robot_state_publisher
  joint_state_publisher
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    urdf
    xacro
    gazebo_ros
    robot_state_publisher
    joint_state_publisher
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# Install urdf files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)

# Install config files if any
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#   PATTERN ".svn" EXCLUDE
# )