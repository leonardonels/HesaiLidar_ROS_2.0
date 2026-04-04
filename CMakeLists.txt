cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

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

if(($ENV{ROS_DISTRO} STREQUAL "kilted"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif(($ENV{ROS_DISTRO} STREQUAL "jazzy"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

#========================
# Dependencies Setup
#========================

find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
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
find_package(barq REQUIRED)

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
ament_target_dependencies(hesai_ros_driver_node
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "barq"
)

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
