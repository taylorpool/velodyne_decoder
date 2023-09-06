cmake_minimum_required(VERSION 3.25)
project(velodyne_decoder
  VERSION 0.1.0
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(VelodyneDecoder
  STATIC
  src/velodyne_decoder.cpp
  src/vlp16.cpp
  src/vlp32c.cpp
)
target_sources(VelodyneDecoder
  PUBLIC
  FILE_SET velodyne_decoder_headers
  TYPE HEADERS
  BASE_DIRS include
  FILES
    include/velodyne_decoder/point.hpp
    include/velodyne_decoder/velodyne_decoder.hpp
    include/velodyne_decoder/vlp16.hpp
    include/velodyne_decoder/vlp32c.hpp
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

  add_executable(vlp16_node
    src/vlp16_node.cpp
  )
  target_link_libraries(vlp16_node
    VelodyneDecoder_ROS1
  )

  add_executable(vlp32c_node
    src/vlp32c_node.cpp
  )
  target_link_libraries(vlp32c_node
    VelodyneDecoder_ROS1
  )
endif()

install(TARGETS VelodyneDecoder
  EXPORT VelodyneDecoderTargets
  ARCHIVE
  LIBRARY
  RUNTIME
  FILE_SET velodyne_decoder_headers
)
install(EXPORT VelodyneDecoderTargets
  NAMESPACE VelodyneDecoder::
  DESTINATION lib/cmake/VelodyneDecoder
)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/VelodyneDecoderConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfig.cmake"
  INSTALL_DESTINATION lib/cmake/VelodyneDecoder
)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfigVersion.cmake"
  COMPATIBILITY ExactVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/VelodyneDecoderConfigVersion.cmake"
  DESTINATION lib/cmake/VelodyneDecoder
)
