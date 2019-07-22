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
find_package(geometry_msgs REQUIRED)
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
include_directories(include)
#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})

################################################################################
# Build
################################################################################
add_executable(open_manipulator_x_teleop_joystick src/open_manipulator_x_teleop_joystick.cpp)
ament_target_dependencies(open_manipulator_x_teleop_joystick "geometry_msgs" "rclcpp" "sensor_msgs" "open_manipulator_msgs")

add_executable(open_manipulator_x_teleop_keyboard src/open_manipulator_x_teleop_keyboard.cpp)
ament_target_dependencies(open_manipulator_x_teleop_keyboard "geometry_msgs" "rclcpp" "sensor_msgs" "open_manipulator_msgs")


#target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME})
#set_target_properties(${PROJECT_NAME}
#  PROPERTIES OUTPUT_NAME open_manipulator_x_teleop PREFIX "")

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_teleop_joystick open_manipulator_x_teleop_keyboard
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################

ament_package()
