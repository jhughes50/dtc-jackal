cmake_minimum_required(VERSION 3.0.2)
project(tdd2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
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
  UTMFix.msg
  TDDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  vision_msgs
  tdd2
)

catkin_package()

catkin_install_python(PROGRAMS
   scripts/detection_and_gps.py
   scripts/sync.py
   scripts/selector.py
   scripts/image_repub.py
   scripts/center_img_node.py
   scripts/rectifier.py
   test/test_localization.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

