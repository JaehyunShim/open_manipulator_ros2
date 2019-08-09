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

#include "open_manipulator_x_teleop/open_manipulator_x_teleop_joystick.h"

namespace open_manipulator_x_teleop_joystick
{

OpenManipulatorXTeleopJoystick::OpenManipulatorXTeleopJoystick()
: Node("open_manipulator_x_teleop_joystick")
{
  /*****************************************************************************
  ** Initialise joint angle and kinematic position size 
  *****************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /*****************************************************************************
  ** Initialise Subscribers
  *****************************************************************************/
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&OpenManipulatorXTeleopJoystick::jointStatesCallback, this, std::placeholders::_1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "kinematics_pose", 10, std::bind(&OpenManipulatorXTeleopJoystick::kinematicsPoseCallback, this, std::placeholders::_1));
  joy_command_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&OpenManipulatorXTeleopJoystick::joyCallback, this, std::placeholders::_1));

  /*****************************************************************************
  ** Initialise Clients
  *****************************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

  RCLCPP_INFO(this->get_logger(), "OpenManipulator Initialised");
}

OpenManipulatorXTeleopJoystick::~OpenManipulatorXTeleopJoystick() {}

/*****************************************************************************
** Callback Functions
*****************************************************************************/
void OpenManipulatorXTeleopJoystick::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorXTeleopJoystick::kinematicsPoseCallback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorXTeleopJoystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if(msg->axes.at(1) >= 0.9) setGoal("x+");
  else if(msg->axes.at(1) <= -0.9) setGoal("x-");
  else if(msg->axes.at(0) >=  0.9) setGoal("y+");
  else if(msg->axes.at(0) <= -0.9) setGoal("y-");
  else if(msg->buttons.at(3) == 1) setGoal("z+");
  else if(msg->buttons.at(0) == 1) setGoal("z-");
  else if(msg->buttons.at(5) == 1) setGoal("home");
  else if(msg->buttons.at(4) == 1) setGoal("init");

  if(msg->buttons.at(2) == 1) setGoal("gripper close");
  else if(msg->buttons.at(1) == 1) setGoal("gripper open");
}

/*****************************************************************************
** Callback Functions and Relevant Functions
*****************************************************************************/
void OpenManipulatorXTeleopJoystick::setGoal(const char* str)
{
  std::vector<double> goalPose;  goalPose.resize(3,0);
  std::vector<double> goalJoint; goalJoint.resize(4,0);

  if(str == "x+")
  {
    printf("increase(++) x axis in cartesian space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "x-")
  {
    printf("decrease(--) x axis in cartesian space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "y+")
  {
    printf("increase(++) y axis in cartesian space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "y-")
  {
    printf("decrease(--) y axis in cartesian space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "z+")
  {
    printf("increase(++) z axis in cartesian space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "z-")
  {
    printf("decrease(--) z axis in cartesian space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "gripper open")
  {
    printf("open gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(str == "gripper close")
  {
    printf("close gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }
  else if(str == "home")
  {
    printf("home pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(str == "init")
  {
    printf("init pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

bool OpenManipulatorXTeleopJoystick::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  RCLCPP_ERROR(this->get_logger(), "No Response :(");
  return false;
}

bool OpenManipulatorXTeleopJoystick::setToolControl(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  RCLCPP_ERROR(this->get_logger(), "No Response :(");
  return false;
}

bool OpenManipulatorXTeleopJoystick::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;


  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  RCLCPP_ERROR(this->get_logger(), "No Response :(");
  return false;
}

}  // namespace open_manipulator_x_teleop_joystick

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<open_manipulator_x_teleop_joystick::OpenManipulatorXTeleopJoystick>());

  rclcpp::shutdown();

  return 0;
}
