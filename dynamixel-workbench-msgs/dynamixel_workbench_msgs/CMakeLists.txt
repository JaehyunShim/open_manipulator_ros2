################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_msgs)

#C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_generation REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  AX.msg
  EX.msg
  MX.msg
  MX2.msg
  MX2Ext.msg
  MXExt.msg
  PRO.msg
  PROExt.msg
  RX.msg
  XH.msg
  XL.msg
  XL320.msg
  XM.msg
  XMExt.msg
  DynamixelState.msg
  DynamixelStateList.msg
  DynamixelInfo.msg
  DynamixelLoadInfo.msg
)

add_service_files(
  FILES
  GetDynamixelInfo.srv
  DynamixelCommand.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
