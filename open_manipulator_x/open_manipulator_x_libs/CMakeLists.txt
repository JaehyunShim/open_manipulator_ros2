################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_libs)

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
find_package(cmake_modules REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  include
)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(dependencies_lib
  "cmake_modules"
  "dynamixel_workbench_toolbox"
  "rclcpp"
  "robotis_manipulator"
  "EIGEN3"
)

add_library(open_manipulator_x_libs
  "src/custom_trajectory.cpp"
  "src/dynamixel.cpp"
  "src/kinematics.cpp"
  "src/open_manipulator_x.cpp"
)
ament_target_dependencies(open_manipulator_x_libs ${dependencies_lib})
target_link_libraries(open_manipulator_x_libs
  ${rclcpp_manipulator_LIBRARIES}
  ${robotis_manipulator_LIBRARIES}
  ${dynamixel_workbench_toolbox_LIBRARIES}
  ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS open_manipulator_x_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
ament_export_dependencies(cmake_modules)
ament_export_dependencies(dynamixel_workbench_toolbox)
ament_export_dependencies(rclcpp)
ament_export_dependencies(robotis_manipulator)
ament_export_dependencies(Eigen3)
ament_export_libraries(open_manipulator_x_libs)
ament_package()
