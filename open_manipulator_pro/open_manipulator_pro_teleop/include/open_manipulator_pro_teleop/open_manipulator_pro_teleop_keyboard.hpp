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

#ifndef OPEN_MANIPULATOR_PRO_TELEOP_KEYBOARD_HPP
#define OPEN_MANIPULATOR_PRO_TELEOP_KEYBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define PI 3.141592
#define NUM_OF_JOINT 6
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

namespace open_manipulator_pro_teleop_keyboard
{
class OpenManipulatorProTeleopKeyboard : public rclcpp::Node
{
 public:
  OpenManipulatorProTeleopKeyboard();
  virtual ~OpenManipulatorProTeleopKeyboard();

 private:
  /*****************************************************************************
  ** Position in Joint Space and Task Space, and Whether with Gripper
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  bool use_gripper_;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub_;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);

  bool set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time);
  bool set_tool_control(std::vector<double> joint_angle);
  bool set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);

  /*****************************************************************************
  ** ROS  Clients
  *****************************************************************************/
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_client_;

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  void set_goal(char ch);
  void print_text();
  std::vector<double> get_present_joint_angle();
  std::vector<double> get_present_kinematics_pose();
  struct termios oldt_;
  void restore_terminal_settings();
  void disable_waiting_for_enter();
  rclcpp::TimerBase::SharedPtr timer_;
  void display_callback(); 
};
}  // namespace open_manipulator_pro_teleop_keyboard
#endif  // OPEN_MANIPULATOR_PRO_TELEOP_KEYBOARD_HPP
