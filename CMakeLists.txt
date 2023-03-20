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

option(BUILD_ROS_VelodyneDecoder "Build ROS Package" OFF)
if(${BUILD_ROS_VelodyneDecoder})
  find_package(catkin REQUIRED COMPONENTS velodyne_msgs)

  catkin_package(
    CATKIN_DEPENDS
    velodyne_msgs
  )

  add_library(velodyne_decoder_ROS
    STATIC
    src/velodyne_decoder_ros.cpp
  )
  target_include_directories(velodyne_decoder_ROS
    PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(velodyne_decoder_ROS
    velodyne_decoder
    ${catkin_LIBRARIES}
  )
endif()