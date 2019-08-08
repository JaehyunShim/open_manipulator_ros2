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

#ifndef OPEN_MANIPULATOR_X_GAZEBO_CONTROLLER_H
#define OPEN_MANIPULATOR_X_GAZEBO_CONTROLLER_H

#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>


#include "open_manipulator_x_libs/open_manipulator_x.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/srv/apply_joint_effort.hpp"
#include "gazebo_msgs/srv/joint_request.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "../include/open_manipulator_x_gazebo_controller/pid.h"


namespace open_manipulator_x_gazebo_controller
{

class OpenManipulatorXGazeboController : public rclcpp::Node
{

 public:
  OpenManipulatorXGazeboController();
  ~OpenManipulatorXGazeboController();

 private:

  // Related robotis_manipulator
  OpenManipulator open_manipulator_;
  std::vector<double> goal_joint_position_;
  std::vector<double> present_joint_position_;
  std::vector<double> goal_joint_effort_;
  std::vector<double> prev_goal_joint_effort_;

  PID pid = PID(0.1, 100, -100, 1.0, 0.40, 0.0);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_joint_position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr present_joint_position_sub_;
  rclcpp::Client<gazebo_msgs::srv::ApplyJointEffort>::SharedPtr goal_joint_effort_client_;
  rclcpp::Client<gazebo_msgs::srv::JointRequest>::SharedPtr clear_joint_effort_client_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr goal_joint_position_pub_;


  void goalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void setPosition();
  void setEffort();
  void convertPositionToEffort();

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace open_manipulator_x_gazebo_controller

#endif //OPEN_MANIPULATOR_X_GAZEBO_CONTROLLER_H
