cmake_minimum_required(VERSION 3.0.2)
project(amm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../eeyore/cmake")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  grid_map_ros
  grid_map_msgs
  mavros_msgs
)

find_package(OpenCV 4 REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smm
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs grid_map_ros grid_map_msgs mavros_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TBB_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/matrix.cpp
  src/pixel_localizer.cpp
)

add_executable(mapper src/handler.cpp src/node.cpp)
target_link_libraries(mapper
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  TBB::tbb
  pthread
  Eigen3::Eigen
)

add_executable(grid_map_test test/test.cpp)
target_link_libraries(grid_map_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  TBB::tbb
  pthread
  Eigen3::Eigen
)
