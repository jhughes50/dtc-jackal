cmake_minimum_required(VERSION 3.0.2)
project(waypoint_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  sensor_msgs
  image_transport
  tf
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

catkin_install_python(PROGRAMS
    scripts/waypoint_nav_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
