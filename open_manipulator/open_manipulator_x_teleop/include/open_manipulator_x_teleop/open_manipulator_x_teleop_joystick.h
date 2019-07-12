/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef OPEN_MANIPULATOR_X_TELEOP_JOYSTICK_H
#define OPEN_MANIPULATOR_X_TELEOP_JOYSTICK_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#include <termios.h>
#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <string>

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

namespace open_manipulator_x_teleop_joystick
{

class OpenManipulatorXTeleopJoystick : public rclcpp::Node
{

public:
  OpenManipulatorXTeleopJoystick();
  virtual ~OpenManipulatorXTeleopJoystick();


  // void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  // void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);

  // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // int64_t enable_button;
  // int64_t enable_turbo_button;

  // std::map<std::string, int64_t> axis_linear_map;
  // std::map<std::string, std::map<std::string, double>> scale_linear_map;

  // std::map<std::string, int64_t> axis_angular_map;
  // std::map<std::string, std::map<std::string, double>> scale_angular_map;

  // bool sent_disable_msg;

  /*****************************************************************************
  ** Position in Joint Space and Task Space
  *****************************************************************************/
  std::vector<double> present_joint_angle;
  std::vector<double> present_kinematic_position;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);
  void joyCallback2(const sensor_msgs::msg::Joy::SharedPtr msg);

  void setGoal(const char *str);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** ROS  Clients
  *****************************************************************************/


private:
  struct Impl; 
  Impl* pimpl_;

  std::shared_ptr<rclcpp::Node> node_ = nullptr;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_sub;

  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_client;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client;

};

}  // namespace open_manipulator_x_teleop_joystick

#endif  // OPEN_MANIPULATOR_X_TELEOP_JOYSTICK_H
