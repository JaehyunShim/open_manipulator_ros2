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

#ifndef OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_H
#define OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

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

namespace open_manipulator_x_teleop_keyboard
{

class OpenManipulatorXTeleopKeyboard : public rclcpp::Node
{

public:
  OpenManipulatorXTeleopKeyboard();
  virtual ~OpenManipulatorXTeleopKeyboard();

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace open_manipulator_x_teleop_keyboard

#endif  // OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_H
