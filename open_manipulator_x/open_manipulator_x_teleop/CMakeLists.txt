################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_teleop)

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
find_package(sensor_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

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
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(open_manipulator_msgs)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(open_manipulator_x_teleop_joystick src/open_manipulator_x_teleop_joystick.cpp)
ament_target_dependencies(open_manipulator_x_teleop_joystick "rclcpp" "sensor_msgs" "open_manipulator_msgs")
target_link_libraries(open_manipulator_x_teleop_joystick ${rclcpp_LIBRARIES})

add_executable(open_manipulator_x_teleop_keyboard src/open_manipulator_x_teleop_keyboard.cpp)
ament_target_dependencies(open_manipulator_x_teleop_keyboard "rclcpp" "sensor_msgs" "open_manipulator_msgs")
target_link_libraries(open_manipulator_x_teleop_keyboard ${rclcpp_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_teleop_joystick open_manipulator_x_teleop_keyboard
#  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}  #???? 
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
