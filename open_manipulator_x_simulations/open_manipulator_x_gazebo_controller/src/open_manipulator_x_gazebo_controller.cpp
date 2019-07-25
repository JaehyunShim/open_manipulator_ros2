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

#include "../include/open_manipulator_x_gazebo_controller/open_manipulator_x_gazebo_controller.h"


#define NUM_OF_JOINT 4
#define NUM_OF_TOOL 1


using namespace open_manipulator_x_gazebo_controller;
using namespace std::placeholders;



OpenManipulatorXGazeboController::OpenManipulatorXGazeboController()
: Node("open_manipulator_x_gazebo_controller")
{
  // From Controller
  goal_joint_position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&OpenManipulatorXGazeboController::goalJointCallback, this, _1));

  // From Gazebo
  present_joint_position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "gazebo/joint_states", 10, std::bind(&OpenManipulatorXGazeboController::jointStatesCallback, this, _1));

  // To Gazebo
  goal_joint_effort_client_ = this->create_client<gazebo_msgs::srv::ApplyJointEffort>("apply_joint_effort");
}

OpenManipulatorXGazeboController::~OpenManipulatorXGazeboController()
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Gazebo Controller Terminated");
}

void OpenManipulatorXGazeboController::goalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT + NUM_OF_TOOL); // Joint + Tool
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if      (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper")) temp_angle.at(4) = (msg->position.at(i));
  }
  goal_joint_position_ = temp_angle;  
}

void OpenManipulatorXGazeboController::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT + 1); // Joint + Tool
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if      (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper")) temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_position_ = temp_angle;  

  bool result;
  result = setEffort();  
  if (result == false) rclcpp::shutdown();
}

bool OpenManipulatorXGazeboController::setEffort()
{
  convertPositionToEffort();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();
  joints_name.reserve(joints_name.size() + tools_name.size());
  joints_name.insert(joints_name.end(), tools_name.begin(), tools_name.end());

  for(uint8_t i = 0; i < joints_name.size(); i ++)
  {
    auto request = std::make_shared<gazebo_msgs::srv::ApplyJointEffort::Request>();
    request->joint_name = joints_name.at(i);
    request->effort = goal_joint_effort_.at(i);
    request->start_time.nanosec = 0;
    request->duration.nanosec = 10000;
    
    using ServiceResponseFuture = rclcpp::Client<gazebo_msgs::srv::ApplyJointEffort>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        return result->success;
    };
    auto future_result = goal_joint_effort_client_->async_send_request(request, response_received_callback);

    RCLCPP_ERROR(this->get_logger(), "Failed :(");
    return false;
  }
}

void OpenManipulatorXGazeboController::convertPositionToEffort()
{
  std::vector<double> temp_angle;
  // temp_angle = goal_joint_position_ - present_joint_position_;
  for(uint8_t i = 0; i < goal_joint_position_.size(); i ++)
    // temp_angle.at(i) = goal_joint_position_.at(i) - present_joint_position_.at(i);
    temp_angle.at(i) = 2.0;

  // goal_effort_ = (pid) temp_angle;
  goal_joint_effort_ = temp_angle;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<open_manipulator_x_gazebo_controller::OpenManipulatorXGazeboController>());

  rclcpp::shutdown();

  return 0;
}
