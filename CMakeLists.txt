cmake_minimum_required(VERSION 2.8.3)
project(dataset_publisher)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV REQUIRED )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrpt_ros_tutorial
#  CATKIN_DEPENDS mrpt_navigation roscpp std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_DIRS}
)

add_executable(mono_kitti_publisher src/mono_kitti_publisher.cpp)
target_link_libraries(mono_kitti_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(stereo_kitti_publisher src/stereo_kitti_publisher.cpp)
target_link_libraries(stereo_kitti_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  pthread
)

add_executable(stereo_EuRoc_publisher src/stereo_EuRoc_publisher.cpp)
target_link_libraries(stereo_EuRoc_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  pthread
)


