cmake_minimum_required(VERSION 3.0.2)
project(mtts_can_ros)

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
  message_generation
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

catkin_install_python(PROGRAMS
    scripts/predict_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
