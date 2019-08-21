################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_gazebo_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(open_manipulator_x_libs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(include)
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${open_manipulator_x_libs_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/pid.cpp)
ament_target_dependencies(${PROJECT_NAME} 
  "rclcpp" 
  "sensor_msgs" 
  "trajectory_msgs" 
  "gazebo_msgs"
  "open_manipulator_x_libs"
)
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES} ${open_manipulator_x_libs_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
#  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(trajectory_msgs)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(open_manipulator_x_libs)
ament_package()
