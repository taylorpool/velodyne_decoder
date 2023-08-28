cmake_minimum_required(VERSION 3.25)
project(velodyne_decoder
  VERSION 0.1.0
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_INSTALL_PREFIX "")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
add_library(VelodyneDecoder
  STATIC
  src/velodyne_decoder.cpp
)
target_sources(VelodyneDecoder
  PUBLIC
  FILE_SET velodyne_decoder_headers
  TYPE HEADERS
  BASE_DIRS include
  FILES
    include/velodyne_decoder/velodyne_decoder.hpp
)
target_link_libraries(VelodyneDecoder
  Eigen3::Eigen
)

install(TARGETS VelodyneDecoder
  EXPORT VelodyneDecoderTargets
  ARCHIVE
  LIBRARY
  RUNTIME
  FILE_SET velodyne_decoder_headers
)
install(EXPORT VelodyneDecoderTargets
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VelodyneDecoder"
)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/VelodyneDecoderConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VelodyneDecoder"
)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfigVersion.cmake"
  COMPATIBILITY ExactVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VelodyneDecoder"
)

option(VELODYNE_DECODER_BUILD_ROS1 "Build ROS1 Package" OFF)
if(VELODYNE_DECODER_BUILD_ROS1)
  find_package(catkin REQUIRED 
    COMPONENTS
      roscpp
      sensor_msgs
      velodyne_msgs
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
  target_sources(VelodyneDecoder_ROS1
    PUBLIC
    FILE_SET velodyne_decoder_ros1_headers
    TYPE HEADERS
    BASE_DIRS include
    FILES
      include/velodyne_decoder/velodyne_decoder_ros.hpp
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