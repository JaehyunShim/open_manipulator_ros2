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
    "controller/joint_states", 10, std::bind(&OpenManipulatorXGazeboController::goalJointCallback, this, _1));

  // From Gazebo
  present_joint_position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&OpenManipulatorXGazeboController::jointStatesCallback, this, _1));

  // To Gazebo
  goal_joint_position_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/demo/set_trajectory_demo", 10);


  goal_joint_effort_client_ = this->create_client<gazebo_msgs::srv::ApplyJointEffort>("/apply_joint_effort");
  clear_joint_effort_client_ = this->create_client<gazebo_msgs::srv::JointRequest>("/clear_joint_forces");

  goal_joint_position_.resize(NUM_OF_JOINT + 1); // Joint + Tool
  present_joint_position_.resize(NUM_OF_JOINT + 1); // Joint + Tool
  goal_joint_effort_.resize(NUM_OF_JOINT + 1); // Joint + Tool
  prev_goal_joint_effort_.resize(NUM_OF_JOINT + 1); // Joint + Tool


  auto period = std::chrono::milliseconds(1); 
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period), 
    std::bind(&OpenManipulatorXGazeboController::setPosition, this));
  // auto period = std::chrono::milliseconds(1); 
  // timer_ = this->create_wall_timer(
  //   std::chrono::duration_cast<std::chrono::milliseconds>(period), 
  //   std::bind(&OpenManipulatorXGazeboController::setEffort, this));
}

OpenManipulatorXGazeboController::~OpenManipulatorXGazeboController()
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Gazebo Controller Terminated");
}

void OpenManipulatorXGazeboController::goalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT + 1); // Joint + Tool
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
}
void OpenManipulatorXGazeboController::setPosition()
{
  trajectory_msgs::msg::JointTrajectory msg;

  // msg.joint_names.resize(1);
  // msg.points.resize(1);
  // msg.points[0].positions.resize(1);
  // std::string joint_name("joint2");
  // msg.joint_names.at(0)= joint_name;
  // msg.points[0].positions.at(0) = 0.0; 
// header: {frame_id: world}
  // msg.header.frame_id.clear();
  // msg.header.frame_id.push_back("world");
  msg.header.frame_id = "world";
  msg.joint_names.resize(4);
  msg.joint_names[0] = "joint1";
  msg.joint_names[1] = "joint2";
  msg.joint_names[2] = "joint3";
  msg.joint_names[3] = "joint4";
  // msg.joint_names.push_back("joint2");
  msg.points.resize(1);
  msg.points[0].positions.resize(msg.joint_names.size(), 1.0);
  msg.points[0].positions[0] = 0.0;
  msg.points[0].positions[1] = 0.0;
  msg.points[0].positions[2] = 0.0;
  msg.points[0].positions[3] = 0.0;
  // msg.points[0].positions.push_back(0.0);

  goal_joint_position_pub_->publish(msg);
}

void OpenManipulatorXGazeboController::setEffort()
{
  convertPositionToEffort();

  // auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  // auto tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();
  // joints_name.reserve(joints_name.size() + tools_name.size());
  // joints_name.insert(joints_name.end(), tools_name.begin(), tools_name.end());

  // joints_name.at(1);
  std::vector<std::string> joints_name;
  joints_name.push_back("joint1");
  joints_name.push_back("joint2");
  joints_name.push_back("joint3");
  joints_name.push_back("joint4");
  joints_name.push_back("gripper");

  for(uint8_t i = 0; i < NUM_OF_JOINT+1; i ++)
  {
    auto request = std::make_shared<gazebo_msgs::srv::JointRequest::Request>();
    request->joint_name = joints_name.at(i);

    RCLCPP_INFO(this->get_logger(), joints_name.at(i));
    using ServiceResponseFuture = rclcpp::Client<gazebo_msgs::srv::JointRequest>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
    };
    auto future_result = clear_joint_effort_client_->async_send_request(request, response_received_callback);
  }

  for(uint8_t i = 0; i < NUM_OF_JOINT+1; i ++)
  {
    auto request = std::make_shared<gazebo_msgs::srv::ApplyJointEffort::Request>();
    request->joint_name = joints_name.at(i);
    // request->joint_name = "joint2";
    request->effort = goal_joint_effort_.at(i);
    // request->effort = goal_joint_effort_.at(i) - prev_goal_joint_effort_.at(i);
    // prev_goal_joint_effort_.at(i) = request->effort;
    // request->effort = -100.0;
    request->start_time.sec = 0;
    request->start_time.nanosec = 0;
    request->duration.sec = 1000;
    request->duration.nanosec = 0;
    
    RCLCPP_INFO(this->get_logger(), joints_name.at(i));
    using ServiceResponseFuture = rclcpp::Client<gazebo_msgs::srv::ApplyJointEffort>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        // RCLCPP_INFO(this->get_logger(), "Succeeded :)");
        // return result->success;
        // return;
    };
    auto future_result = goal_joint_effort_client_->async_send_request(request, response_received_callback);

    // RCLCPP_ERROR(this->get_logger(), "Failed :(");
    // return false;
    // return;
  }
    // return true;
}

void OpenManipulatorXGazeboController::convertPositionToEffort()
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT + 1); // Joint + Tool
  double temp_angle2;
  // temp_angle = goal_joint_position_ - present_joint_position_;
  for(uint8_t i = 0; i < NUM_OF_JOINT + 1; i ++)
  {

    // log::info("joint no", i+1);
    // log::info("goal_joint_position_", goal_joint_position_.at(i));
    // log::info("present_joint_position_", present_joint_position_.at(i));

    double val = (goal_joint_position_.at(i) - present_joint_position_.at(i));
    // for (int i = 0; i < 10; i++) {
        double inc = pid.calculate(0, val);
        printf("val: %f  inc: %f \n", val, inc);
        // val += inc;
    // }
    temp_angle.at(i) = -inc;
    // printf("val: %f \n", val);

    // temp_angle.at(i) = (goal_joint_position_.at(i) - present_joint_position_.at(i))*2;
    // RCLCPP_ERROR(this->get_logger(), goal_joint_position_.at(i));
    // RCLCPP_ERROR(this->get_logger(), present_joint_position_.at(i));
    // temp_angle.at(i) = -1.5;
  //   // temp_angle2 = goal_joint_position_.at(i) - present_joint_position_.at(i);
  //   // temp_angle.push_back(temp_angle2*10);
  //   // temp_angle.push_back(10);
  }

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
