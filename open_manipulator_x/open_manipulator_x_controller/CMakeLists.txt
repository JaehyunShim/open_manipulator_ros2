################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(open_manipulator_x_libs REQUIRED)
find_package(cmake_modules REQUIRED)
#find_package(moveit_msgs REQUIRED)
#find_package(moveit_core REQUIRED)
#find_package(moveit_ros_planning REQUIRED) 
#find_package(moveit_ros_planning_interface REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(trajectory_msgs)
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(robotis_manipulator)
ament_export_dependencies(open_manipulator_x_libs)
ament_export_dependencies(cmake_modules)
#ament_export_dependencies(moveit_core)
#ament_export_dependencies(moveit_msgs)
#ament_export_dependencies(moveit_ros_planning)
#ament_export_dependencies(moveit_ros_planning_interface)

################################################################################
# Build
################################################################################
include_directories(include)
include_directories(${rclcpp_INCLUDE_DIRS}
                    ${std_INCLUDE_DIRS}
                    ${sensor_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${trajectory_msgs_INCLUDE_DIRS}
                    ${open_manipulator_msgs_INCLUDE_DIRS}
                    ${robotis_manipulator_INCLUDE_DIRS}
                    ${open_manipulator_x_libs_INCLUDE_DIRS}
                    ${cmake_modules_INCLUDE_DIRS})
#                    ${moveit_core_INCLUDE_DIRS}
#                    ${moveit_msgs_INCLUDE_DIRS}
#                    ${moveit_ros_planning_INCLUDE_DIRS}
#                    ${moveit_ros_planning_interface_INCLUDE_DIRS}

add_executable(open_manipulator_x_controller src/open_manipulator_x_controller.cpp)
ament_target_dependencies(open_manipulator_x_controller 
  "rclcpp" 
  "std_msgs" 
  "sensor_msgs" 
  "geometry_msgs" 
  "trajectory_msgs" 
  "open_manipulator_msgs"
  "robotis_manipulator" 
  "open_manipulator_x_libs" 
  "cmake_modules"
)
target_link_libraries(open_manipulator_x_controller 
                      ${rclcpp_LIBRARIES}
                      ${robotis_manipulator_LIBRARIES}
                      ${open_manipulator_x_libs_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_controller
#  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/create_udev_rules
#  DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES 99-open-manipulator-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_package()
