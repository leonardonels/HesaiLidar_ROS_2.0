cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall)

#=======================================
# Options and configurations
#=======================================
option(ENABLE_BARQ "Enable BARQ for high-performance packet processing" OFF)
option(ENABLE_VERBOSE "Enable verbose logging for debugging purposes" OFF)

#=======================================
# Verbose logging
#=======================================
if(ENABLE_VERBOSE)
  add_definitions(-DVERBOSE_LOGGING)
  message(
    "============================================================\n"
    "-- VERBOSE LOGGING is ENABLED\n"
    "============================================================\n"
  )
else()
  message(
    "============================================================\n"
    "-- VERBOSE LOGGING is DISABLED - IF YOU WANT TO ENABLE VERBOSE LOGGING, PLEASE SET -DENABLE_VERBOSE=ON WHEN BUILDING THIS PACKAGE\n"
    "============================================================\n"
  )
endif()

#=======================================
# BARQ support
#=======================================
if($ENV{ENABLE_BARQ})
  set(ENABLE_BARQ ON)
endif()

if(ENABLE_BARQ)
  find_package(barq QUIET)
  if (NOT barq_FOUND)
    message(FATAL_ERROR 
      "============================================================\n"
      "ENABLE_BARQ is ON, but the BARQ package was not found.\n"
      "This package is not avaiable on rosdep or apt.\n"
      "You must clone and built it manually from:\n"
      "    cd ~/ros2_ws/src\n"
      "    git clone https://github.com/leonardonels/barq.git\n"
      "    cd ~/ros2_ws && colcon build\n"
      "Then rebuild this package with:\n"
      "    cd ~/ros2_ws && ENABLE_BARQ=1 colcon build --packages-up-to hesai_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release\n"
      "============================================================\n"
    )
  endif()
  message(
    "============================================================\n"
    "-- BARQ is ENABLED\n"
    "============================================================\n"
  )
  add_definitions(-DENABLE_BARQ)
else()
  message( 
    "============================================================\n"
    "-- BARQ is DISABLED - IF YOU WANT TO ENABLE BARQ, PLEASE SET ENV ENABLE_BARQ=1 BEFORE BUILDING THIS PACKAGE\n"
    "============================================================\n"
  )
endif()

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  11)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs")

# uncomment to use gpu acceleration
find_package(CUDA QUIET)

if(${CUDA_FOUND})
  message(=============================================================)
  message("-- CUDA_FOUND is True")
  message(=============================================================)

  project(CommonModules LANGUAGES CXX CUDA)
	set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
	set(CUDA_SEPARABLE_COMPILATION ON)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;)
	link_directories($ENV{CUDA_PATH}/lib/x64)

  add_definitions(-DUSE_CUDA)
  set(FIND_CUDA ON CACHE BOOL "Use Cuda")
else()
  message(=============================================================)
  message("-- CUDA NOT FOUND")
  message(=============================================================)
endif()

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#========================
# Project details / setup
#========================
set(PROJECT_NAME hesai_ros_driver)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

#========================
# Dependencies Setup
#========================

find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UdpPacket.msg"
  "msg/UdpFrame.msg"
  "msg/Firetime.msg"
  "msg/Ptp.msg"
  "msg/LossPacket.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

#Others#
find_package(yaml-cpp REQUIRED)

#Include directories#
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src/driver/HesaiLidar_SDK_2.0/libhesai
)

#Driver core#
set(DISENABLE_TEST_CC ON CACHE BOOL "DISENABLE_TEST_CC")
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

#========================
# Build Setup
#========================

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(LIB_TARGETS
    hesai_sdk_lib
    log_lib
    lidar_lib
    ptcClient_lib
    source_lib
    platutils_lib
    serialClient_lib
    udpParser_lib
    )

if(${CUDA_FOUND})
  add_executable(hesai_ros_driver_node
    src/hesai_ros_driver_node.cu
    src/node_manager.cc
    src/driver/source_driver_ros2.cpp
    )
  set(LIB_TARGETS
    ${LIB_TARGETS}
    udpParserGpu_lib
    )
else()
  add_executable(hesai_ros_driver_node
    src/hesai_ros_driver_node.cc
    src/node_manager.cc
    src/driver/source_driver_ros2.cpp
    )
endif()
target_link_libraries(hesai_ros_driver_node
            ${YAML_CPP_LIBRARIES}
            ${Boost_LIBRARIES}
            ${LIB_TARGETS}
            rt
            pthread
)

target_compile_options(hesai_ros_driver_node PRIVATE
  -Wno-class-memaccess
  -Wno-unused-variable
)

target_include_directories(hesai_ros_driver_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

#========================
# ROS2 Install & Package
#========================

find_package(ament_index_cpp REQUIRED)

if(barq_FOUND)
ament_target_dependencies(hesai_ros_driver_node
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
  "barq"
)
else()
ament_target_dependencies(hesai_ros_driver_node
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
)
endif()


if(($ENV{ROS_DISTRO} STREQUAL "kilted") OR ($ENV{ROS_DISTRO} STREQUAL "jazzy") OR ($ENV{ROS_DISTRO} STREQUAL "humble"))
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(hesai_ros_driver_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(hesai_ros_driver_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS
        hesai_ros_driver_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS
        ${LIB_TARGETS}
        DESTINATION lib)

install(DIRECTORY
        include/
        DESTINATION include)

install(DIRECTORY
        launch
        rviz
        config
        DESTINATION share/${PROJECT_NAME})

ament_package()
