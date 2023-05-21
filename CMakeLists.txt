cmake_minimum_required(VERSION 3.16)
project(velodyne_decoder)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

add_library(velodyne_decoder
  STATIC
  src/velodyne_decoder.cpp
)
target_include_directories(velodyne_decoder
  PUBLIC
  include
)

option(VelodyneDecoder_BUILD_ROS1 "Build ROS Package" OFF)
if(VelodyneDecoder_BUILD_ROS1)
  find_package(catkin REQUIRED 
    COMPONENTS roscpp sensor_msgs velodyne_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    velodyne_msgs
    roscpp
    sensor_msgs
  )

  add_library(velodyne_decoder_ROS1
    STATIC
    src/velodyne_decoder_ros.cpp
  )
  target_include_directories(velodyne_decoder_ROS1
    PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(velodyne_decoder_ROS1
    velodyne_decoder
    ${catkin_LIBRARIES}
  )

  add_executable(velodyne_decoder_node
    src/velodyne_decoder_node.cpp
  )
  target_link_libraries(velodyne_decoder_node
    velodyne_decoder_ROS1
  )
endif()