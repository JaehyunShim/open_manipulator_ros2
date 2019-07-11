################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(teleop_twist_joy)

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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
include_directories(include)

################################################################################
# Build
################################################################################
add_executable(${PROJECT_NAME} src/teleop_twist_joy.cpp)
#target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME})
#set_target_properties(${PROJECT_NAME}
#  PROPERTIES OUTPUT_NAME teleop_twist_joy PREFIX "")
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include)

#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################
# Test
################################################################################
#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
ament_package()
