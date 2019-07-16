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
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(trajectory_msgs)
find_package(open_manipulator_msgs)
find_package(robotis_manipulator)
find_package(open_manipulator_x_libs)
find_package(cmake_modules)
#find_package(moveit_msgs)
#find_package(moveit_core)
#find_package(moveit_ros_planning)
#find_package(moveit_ros_planning_interface)
find_package(Boost REQUIRED)

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
# Declare catkin specific configuration to be passed to dependent projects
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
ament_export_dependencies(Boost)

################################################################################
# Build
################################################################################
ament_export_include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(open_manipulator_x_controller src/open_manipulator_x_controller.cpp)
ament_target_dependencies(open_manipulator_x_controller 
  rclcpp std_msgs sensor_msgs geometry_msgs trajectory_msgs open_manipulator_msgs
  robotis_manipulator open_manipulator_x_libs cmake_modules)
target_link_libraries(open_manipulator_x_controller ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_controller
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 99-open-manipulator-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
ament_package()
