################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_manipulator)

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
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

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
ament_export_dependencies(cmake_modules)
ament_export_libraries(robotis_manipulator)

################################################################################
# Build
################################################################################
ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(robotis_manipulator
  src/robotis_manipulator/robotis_manipulator.cpp
  src/robotis_manipulator/robotis_manipulator_common.cpp
  src/robotis_manipulator/robotis_manipulator_log.cpp
  src/robotis_manipulator/robotis_manipulator_trajectory_generator.cpp
  src/robotis_manipulator/robotis_manipulator_manager.cpp
  src/robotis_manipulator/robotis_manipulator_math.cpp
)

#add_dependencies(robotis_manipulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotis_manipulator 
                      ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS robotis_manipulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
ament_package()
