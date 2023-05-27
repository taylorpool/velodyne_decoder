cmake_minimum_required(VERSION 3.16)
project(velodyne_decoder)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(VelodyneDecoder
  STATIC
  src/velodyne_decoder.cpp
)
target_include_directories(VelodyneDecoder
  PUBLIC
  include
)

option(VELODYNE_DECODER_BUILD_ROS1 "Build ROS1 Package" OFF)
if(VELODYNE_DECODER_BUILD_ROS1)
  find_package(catkin REQUIRED 
    COMPONENTS roscpp sensor_msgs velodyne_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    velodyne_msgs
    roscpp
    sensor_msgs
  )

  add_library(VelodyneDecoder_ROS1
    STATIC
    src/velodyne_decoder_ros.cpp
  )
  target_include_directories(VelodyneDecoder_ROS1
    PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(VelodyneDecoder_ROS1
    VelodyneDecoder
    ${catkin_LIBRARIES}
  )

  add_executable(velodyne_decoder_node
    src/velodyne_decoder_node.cpp
  )
  target_link_libraries(velodyne_decoder_node
    VelodyneDecoder_ROS1
  )
endif()