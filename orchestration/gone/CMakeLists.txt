cmake_minimum_required(VERSION 3.0.2)
project(gone)

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


add_message_files(
  FILES
  GroundDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/orchestrator_node.py
    scripts/monitor_node.py
    test/trigger_test.py    
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
