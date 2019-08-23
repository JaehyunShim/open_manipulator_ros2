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

#include "open_manipulator_pro_controller/open_manipulator_pro_controller.hpp"

using namespace open_manipulator_pro_controller;
using namespace std::placeholders;

OpenManipulatorProController::OpenManipulatorProController(std::string usb_port, std::string baud_rate)
: Node("open_manipulator_pro_controller"),
  control_period_(0.010),
  use_platform_(false),
  use_gripper_(false),
  use_moveit_(false)
{
  /************************************************************
  ** Get Parameters
  ************************************************************/
  this->get_parameter_or("control_period", control_period_, 0.010);
  this->get_parameter_or("use_platform", use_platform_, true);
  this->get_parameter_or("use_gripper", use_gripper_, true);
  this->get_parameter_or("use_moveit", use_moveit_, false);

  open_manipulator_pro_.init_open_manipulator_pro(use_platform_, usb_port, baud_rate, 0.010, use_gripper_);

  if (use_platform_ == true)       
    log::info("Succeeded to Initialise OpenManipulator-PRO Controller");
  else if (use_platform_ == false) 
    log::info("Ready to Simulate OpenManipulator-PRO on Gazebo");

  /************************************************************
  ** Initialise ROS Publishers, Subscribers and Servers
  ************************************************************/
  init_publisher();
  init_subscriber();
  init_server();

  // Only if You Have MoveIt! Dependencies
  // open_manipulator_pro_controller_moveit.init_publisher();
  // open_manipulator_pro_controller_moveit.init_subscriber();
  // open_manipulator_pro_controller_moveit.init_server();

  /************************************************************
  ** Start Process and Publish Threads
  ************************************************************/
  auto period = std::chrono::milliseconds(10); 
  process_timer = this->create_wall_timer(
    period, std::bind(&OpenManipulatorProController::process_callback, this));

  publish_timer = this->create_wall_timer(
    period, std::bind(&OpenManipulatorProController::publish_callback, this));
}

OpenManipulatorProController::~OpenManipulatorProController()
{
  log::info("Shutdown the OpenManipulator-PRO Controller");
  open_manipulator_pro_.disableAllActuator();
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorProController::init_publisher()
{
  auto om_tools_name = open_manipulator_pro_.getManipulator()->getAllToolComponentName();

  open_manipulator_pro_states_pub_ = this->create_publisher<open_manipulator_msgs::msg::OpenManipulatorState>("states", 10);

  for (auto const& name:om_tools_name)
  {
    auto pb = this->create_publisher<open_manipulator_msgs::msg::KinematicsPose>("open_manipulator_pro/kinematics_pose", 10);
    open_manipulator_pro_kinematics_pose_pub_.push_back(pb);
  }

  if(use_platform_ == true)
  {
    open_manipulator_pro_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("open_manipulator_pro/joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_pro_.getManipulator()->getAllActiveJointComponentName();
    gazebo_joints_name.reserve(gazebo_joints_name.size() + om_tools_name.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), om_tools_name.begin(), om_tools_name.end());

    for (auto const& name:gazebo_joints_name)
    {
      auto pb = this->create_publisher<std_msgs::msg::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }
}

void OpenManipulatorProController::init_subscriber()
{
  open_manipulator_pro_option_sub_ = this->create_subscription<std_msgs::msg::String>(
    "open_manipulator_pro/option", 10, std::bind(&OpenManipulatorProController::open_manipulator_pro_option_callback, this, _1));
}

void OpenManipulatorProController::init_server()
{
  goal_joint_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path", std::bind(&OpenManipulatorProController::goal_joint_space_path_callback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_pose_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_pose", std::bind(&OpenManipulatorProController::goal_joint_space_path_to_kinematics_pose_callback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_position_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_position", std::bind(&OpenManipulatorProController::goal_joint_space_path_to_kinematics_position_callback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_orientation_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_orientation", std::bind(&OpenManipulatorProController::goal_joint_space_path_to_kinematics_orientation_callback, this, _1, _2, _3));

  goal_task_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path", std::bind(&OpenManipulatorProController::goal_task_space_path_callback, this, _1, _2, _3));
  goal_task_space_path_position_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_position_only", std::bind(&OpenManipulatorProController::goal_task_space_path_position_only_callback, this, _1, _2, _3));
  goal_task_space_path_orientation_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_orientation_only", std::bind(&OpenManipulatorProController::goal_task_space_path_orientation_only_callback, this, _1, _2, _3));

  goal_joint_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present", std::bind(&OpenManipulatorProController::goal_joint_space_path_from_present_callback, this, _1, _2, _3));

  goal_task_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present", std::bind(&OpenManipulatorProController::goal_task_space_path_from_present_callback, this, _1, _2, _3));
  goal_task_space_path_from_present_position_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present_position_only", std::bind(&OpenManipulatorProController::goal_task_space_path_from_present_position_only_callback, this, _1, _2, _3));
  goal_task_space_path_from_present_orientation_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present_orientation_only", std::bind(&OpenManipulatorProController::goal_task_space_path_from_present_orientation_only_callback, this, _1, _2, _3));

  goal_tool_control_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_tool_control", std::bind(&OpenManipulatorProController::goal_tool_control_callback, this, _1, _2, _3));
  set_actuator_state_server_ = this->create_service<open_manipulator_msgs::srv::SetActuatorState>(
    "set_actuator_state", std::bind(&OpenManipulatorProController::set_actuator_state_callback, this, _1, _2, _3));
  goal_drawing_trajectory_server_ = this->create_service<open_manipulator_msgs::srv::SetDrawingTrajectory>(
    "goal_drawing_trajectory", std::bind(&OpenManipulatorProController::goal_drawing_trajectory_callback, this, _1, _2, _3));
}

/*****************************************************************************
** Callback Functions for ROS Subscribers
*****************************************************************************/
void OpenManipulatorProController::open_manipulator_pro_option_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "print_open_manipulator_pro_setting")
    open_manipulator_pro_.printManipulatorSetting();
}

/*****************************************************************************
** Callback Functions for ROS Servers
*****************************************************************************/
void OpenManipulatorProController::goal_joint_space_path_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    target_angle.push_back(req->joint_position.position.at(i));

  open_manipulator_pro_.makeJointTrajectory(target_angle, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_joint_space_path_to_kinematics_pose_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_pro_.makeJointTrajectory(req->end_effector_name, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_joint_space_path_to_kinematics_position_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_pro_.makeJointTrajectory(req->end_effector_name, target_pose.position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_joint_space_path_to_kinematics_orientation_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_pro_.makeJointTrajectory(req->end_effector_name, target_pose.orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  open_manipulator_pro_.makeTaskTrajectory(req->end_effector_name, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_position_only_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Vector3d position;
  position[0] = req->kinematics_pose.pose.position.x;
  position[1] = req->kinematics_pose.pose.position.y;
  position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_pro_.makeTaskTrajectory(req->end_effector_name, position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_orientation_only_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_pro_.makeTaskTrajectory(req->end_effector_name, orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_joint_space_path_from_present_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    target_angle.push_back(req->joint_position.position.at(i));

  open_manipulator_pro_.makeJointTrajectoryFromPresentPosition(target_angle, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_from_present_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_pro_.makeTaskTrajectoryFromPresentPose(req->planning_group, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_from_present_position_only_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Vector3d position;
  position[0] = req->kinematics_pose.pose.position.x;
  position[1] = req->kinematics_pose.pose.position.y;
  position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_pro_.makeTaskTrajectoryFromPresentPose(req->planning_group, position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_task_space_path_from_present_orientation_only_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_pro_.makeTaskTrajectoryFromPresentPose(req->planning_group, orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_tool_control_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    open_manipulator_pro_.makeToolTrajectory(req->joint_position.joint_name.at(i), req->joint_position.position.at(i));

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::set_actuator_state_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res)
{
  if(req->set_actuator_state == true) // enable actuators
  {
    log::println("Wait a second for actuator enable", "GREEN");
    open_manipulator_pro_.enableAllActuator();
  }
  else // disable actuators
  {
    log::println("Wait a second for actuator disable", "GREEN");
    open_manipulator_pro_.disableAllActuator();
  }

  res->is_planned = true;
  return;
}

void OpenManipulatorProController::goal_drawing_trajectory_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res)
{
  try
  {
    if(req->drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req->param[0];  // radius (m)
      draw_circle_arg[1] = req->param[1];  // revolution (rev)
      draw_circle_arg[2] = req->param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;

      open_manipulator_pro_.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req->end_effector_name, p_draw_circle_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req->param[0]; // x axis (m)
      draw_line_arg.kinematic.position(1) = req->param[1]; // y axis (m)
      draw_line_arg.kinematic.position(2) = req->param[2]; // z axis (m)
      void *p_draw_line_arg = &draw_line_arg;

      open_manipulator_pro_.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req->end_effector_name, p_draw_line_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req->param[0];  // radius (m)
      draw_rhombus_arg[1] = req->param[1];  // revolution (rev)
      draw_rhombus_arg[2] = req->param[2];  // start angle position (rad)
      void* p_draw_rhombus_arg = &draw_rhombus_arg;

      open_manipulator_pro_.makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req->end_effector_name, p_draw_rhombus_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req->param[0];  // radius (m)
      draw_heart_arg[1] = req->param[1];  // revolution (rev)
      draw_heart_arg[2] = req->param[2];  // start angle position (rad)
      void* p_draw_heart_arg = &draw_heart_arg;

      open_manipulator_pro_.makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req->end_effector_name, p_draw_heart_arg, req->path_time);
    }
    res->is_planned = true;
    return;
  }
  catch (rclcpp::exceptions::RCLError &e)
  {
    log::error("Failed to Create a Custom Trajectory");
  }
  return;
}

/********************************************************************************
** Functions related to processCallback 
********************************************************************************/
void OpenManipulatorProController::process_callback()   
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time present_time = clock.now();
  this->process(present_time.seconds());
}

void OpenManipulatorProController::process(double time)
{
  open_manipulator_pro_.process_open_manipulator_pro(time, use_platform_, use_gripper_);

  // Only if You Have MoveIt! Dependencies
  // open_manipulator_pro_controller_moveit.moveitTimer(time);
}

/********************************************************************************
** Functions related to publishCallback 
********************************************************************************/
void OpenManipulatorProController::publish_callback()   
{
  if (use_platform_ == true) publish_joint_states();
  else publish_gazebo_command();

  publish_open_manipulator_pro_states();
  publish_kinematics_pose();
}

void OpenManipulatorProController::publish_open_manipulator_pro_states()
{
  open_manipulator_msgs::msg::OpenManipulatorState msg;
  if(open_manipulator_pro_.getMovingState())
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  else
    msg.open_manipulator_moving_state = msg.STOPPED;

  if(open_manipulator_pro_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  open_manipulator_pro_states_pub_->publish(msg);
}

void OpenManipulatorProController::publish_kinematics_pose()
{
  open_manipulator_msgs::msg::KinematicsPose msg;
  auto om_tools_name = open_manipulator_pro_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:om_tools_name)
  {
    KinematicPose pose = open_manipulator_pro_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_pro_kinematics_pose_pub_.at(index)->publish(msg);
    index++;
  }
}

void OpenManipulatorProController::publish_joint_states()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = rclcpp::Clock().now();

  auto joints_name = open_manipulator_pro_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_pro_.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator_pro_.getAllActiveJointValue();
  auto tool_value = open_manipulator_pro_.getAllToolValue();

  for(uint8_t i = 0; i < joints_name.size(); i ++)
  {
    msg.name.push_back(joints_name.at(i));

    msg.position.push_back(joint_value.at(i).position);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }

  for(uint8_t i = 0; i < tool_name.size(); i ++)
  {
    msg.name.push_back(tool_name.at(i));

    msg.position.push_back(tool_value.at(i).position);
    msg.velocity.push_back(0.0f);
    msg.effort.push_back(0.0f);
  }
  open_manipulator_pro_joint_states_pub_->publish(msg);
}

void OpenManipulatorProController::publish_gazebo_command()
{
  JointWaypoint joint_value = open_manipulator_pro_.getAllActiveJointValue();
  JointWaypoint tool_value = open_manipulator_pro_.getAllToolValue();

  for(uint8_t i = 0; i < joint_value.size(); i ++)
  {
    std_msgs::msg::Float64 msg;
    msg.data = joint_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(i)->publish(msg);
  }

  for(uint8_t i = 0; i < tool_value.size(); i ++)
  {
    std_msgs::msg::Float64 msg;
    msg.data = tool_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i)->publish(msg);
  }
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc == 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());
  }
  else
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());

  rclcpp::spin(std::make_unique<open_manipulator_pro_controller::OpenManipulatorProController>(usb_port, baud_rate));

  rclcpp::shutdown();

  return 0;
}
