################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_teleop2)

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
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(open_manipulator_msgs)

################################################################################
# Build
################################################################################
add_executable(open_manipulator_teleop_keyboard src/open_manipulator_teleop_keyboard.cpp)
ament_target_dependencies(open_manipulator_teleop_keyboard "rclcpp" "std_msgs" "sensor_msgs" "open_manipulator_msgs")
#target_link_libraries(open_manipulator_teleop_keyboard ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_joystick src/open_manipulator_teleop_joystick.cpp)
ament_target_dependencies(open_manipulator_teleop_joystick "rclcpp" "std_msgs" "sensor_msgs" "open_manipulator_msgs")
#target_link_libraries(open_manipulator_teleop_joystick ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_teleop_keyboard open_manipulator_teleop_joystick
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()