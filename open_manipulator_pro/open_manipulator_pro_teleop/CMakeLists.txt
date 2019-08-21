################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_pro_teleop)

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
find_package(open_manipulator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "open_manipulator_msgs"
  "rclcpp"
  "sensor_msgs"
)

add_executable(open_manipulator_pro_teleop_joystick src/open_manipulator_pro_teleop_joystick.cpp)
ament_target_dependencies(open_manipulator_pro_teleop_joystick ${dependencies})
target_link_libraries(open_manipulator_pro_teleop_joystick ${rclcpp_LIBRARIES})

add_executable(open_manipulator_pro_teleop_keyboard src/open_manipulator_pro_teleop_keyboard.cpp)
ament_target_dependencies(open_manipulator_pro_teleop_keyboard ${dependencies})
target_link_libraries(open_manipulator_pro_teleop_keyboard ${rclcpp_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_pro_teleop_joystick open_manipulator_pro_teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}  
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_package()
