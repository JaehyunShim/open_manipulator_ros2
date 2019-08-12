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

#include "open_manipulator_x_controller/open_manipulator_x_controller.h"

using namespace open_manipulator_x_controller;
using namespace std::placeholders;

OpenManipulatorXController::OpenManipulatorXController(std::string usb_port, std::string baud_rate)
: Node("open_manipulator_x_controller"),
  control_period_(0.010),
  using_platform_(false)
  // moveit_plan_state_(false),
  // using_moveit_(false),
  // moveit_plan_only_(true),
  // moveit_sampling_time_(0.050f)
{
  /************************************************************
  ** Get Parameters
  ************************************************************/
  this->get_parameter_or("control_period", control_period_, 0.010);
  this->get_parameter_or("use_platform", using_platform_, true);
  // this->get_parameter_or("using_moveit", using_moveit_, false);
  // this->get_parameter_or("moveit_sample_duration", moveit_sampling_time_, 0.050f);
  // std::string planning_group_name;
  // this->get_parameter_or("planning_group_name", planning_group_name, "arm");

  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, 0.010);

  if (using_platform_ == true)       log::info("Succeeded to Initialise OpenManipulator-X Controller");
  else if (using_platform_ == false) log::info("Ready to Simulate OpenManipulator-X on Gazebo");

  // if (using_moveit_ == true)
  // {
  //   move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
  //   log::info("Ready to control " + planning_group_name + " group");
  // }

  /************************************************************
  ** Initialise ROS Publishers, Subscribers and Servers
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();

  /************************************************************
  ** Start Process and Publish Threads
  ************************************************************/
  auto period = std::chrono::milliseconds(10); 
  process_timer = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period), 
    std::bind(&OpenManipulatorXController::processCallback, this));

  auto period2 = std::chrono::milliseconds(10); 
  publish_timer = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period), 
    std::bind(&OpenManipulatorXController::publishCallback, this));
}

OpenManipulatorXController::~OpenManipulatorXController()
{
  log::info("Shutdown the OpenManipulator-X Controller");
  open_manipulator_.disableAllActuator();
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorXController::initPublisher()
{
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  open_manipulator_states_pub_ = this->create_publisher<open_manipulator_msgs::msg::OpenManipulatorState>("states", 10);

  for (auto const& name:om_tools_name)
  {
    auto pb = this->create_publisher<open_manipulator_msgs::msg::KinematicsPose>(name + "/kinematics_pose", 10);
    open_manipulator_kinematics_pose_pub_.push_back(pb);
  }

  if(using_platform_ == true)
  {
    open_manipulator_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("open_manipulator_x/joint_states", 10);
  }
  else
  {
  //   auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  //   gazebo_joints_name.reserve(gazebo_joints_name.size() + om_tools_name.size());
  //   gazebo_joints_name.insert(gazebo_joints_name.end(), om_tools_name.begin(), om_tools_name.end());

  //   for (auto const& name:gazebo_joints_name)
  //   {
  //     auto pb = this->create_publisher<std_msgs::msg::Float64>(name + "_position/command", 10);
  //     gazebo_goal_joint_position_pub_.push_back(pb);
  //   }
  // }
  // if (using_moveit_ == true)
  // {
  //   moveit_update_start_state_pub_ = node_handle_.advertise<std_msgs::Empty>("rviz/moveit/update_start_state", 10);
  }
}

void OpenManipulatorXController::initSubscriber()
{
  open_manipulator_option_sub_ = this->create_subscription<std_msgs::msg::String>(
    "option", 10, std::bind(&OpenManipulatorXController::openManipulatorOptionCallback, this, _1));

  // if (using_moveit_ == true)
  // {
  //   display_planned_path_sub_ = node_handle_.subscribe("/move_group/display_planned_path", 100,
  //                                                      &OpenManipulatorXController::displayPlannedPathCallback, this);
  //   move_group_goal_sub_ = node_handle_.subscribe("/move_group/goal", 100,
  //                                                      &OpenManipulatorXController::moveGroupGoalCallback, this);
  //   execute_traj_goal_sub_ = node_handle_.subscribe("/execute_trajectory/goal", 100,
  //                                                      &OpenManipulatorXController::executeTrajGoalCallback, this);
  // }
}

void OpenManipulatorXController::initServer()
{
  goal_joint_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path", std::bind(&OpenManipulatorXController::goalJointSpacePathCallback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_pose_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_pose", std::bind(&OpenManipulatorXController::goalJointSpacePathToKinematicsPoseCallback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_position_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_position", std::bind(&OpenManipulatorXController::goalJointSpacePathToKinematicsPositionCallback, this, _1, _2, _3));
  goal_joint_space_path_to_kinematics_orientation_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_orientation", std::bind(&OpenManipulatorXController::goalJointSpacePathToKinematicsOrientationCallback, this, _1, _2, _3));

  goal_task_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path", std::bind(&OpenManipulatorXController::goalTaskSpacePathCallback, this, _1, _2, _3));
  goal_task_space_path_position_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_position_only", std::bind(&OpenManipulatorXController::goalTaskSpacePathPositionOnlyCallback, this, _1, _2, _3));
  goal_task_space_path_orientation_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_orientation_only", std::bind(&OpenManipulatorXController::goalTaskSpacePathOrientationOnlyCallback, this, _1, _2, _3));

  goal_joint_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present", std::bind(&OpenManipulatorXController::goalJointSpacePathFromPresentCallback, this, _1, _2, _3));

  goal_task_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present", std::bind(&OpenManipulatorXController::goalTaskSpacePathFromPresentCallback, this, _1, _2, _3));
  goal_task_space_path_from_present_position_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present_position_only", std::bind(&OpenManipulatorXController::goalTaskSpacePathFromPresentPositionOnlyCallback, this, _1, _2, _3));
  goal_task_space_path_from_present_orientation_only_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present_orientation_only", std::bind(&OpenManipulatorXController::goalTaskSpacePathFromPresentOrientationOnlyCallback, this, _1, _2, _3));

  goal_tool_control_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_tool_control", std::bind(&OpenManipulatorXController::goalToolControlCallback, this, _1, _2, _3));
  set_actuator_state_server_ = this->create_service<open_manipulator_msgs::srv::SetActuatorState>(
    "set_actuator_state", std::bind(&OpenManipulatorXController::setActuatorStateCallback, this, _1, _2, _3));
  goal_drawing_trajectory_server_ = this->create_service<open_manipulator_msgs::srv::SetDrawingTrajectory>(
    "goal_drawing_trajectory", std::bind(&OpenManipulatorXController::goalDrawingTrajectoryCallback, this, _1, _2, _3));

  // if (using_moveit_ == true)
  // {
  //   get_joint_position_server_  = priv_node_handle_.advertiseService("moveit/get_joint_position", &OpenManipulatorXController::getJointPositionMsgCallback, this);
  //   get_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/get_kinematics_pose", &OpenManipulatorXController::getKinematicsPoseMsgCallback, this);
  //   set_joint_position_server_  = priv_node_handle_.advertiseService("moveit/set_joint_position", &OpenManipulatorXController::setJointPositionMsgCallback, this);
  //   set_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/set_kinematics_pose", &OpenManipulatorXController::setKinematicsPoseMsgCallback, this);
  // }
}

/*****************************************************************************
** Callback Functions for ROS Subscribers
*****************************************************************************/
void OpenManipulatorXController::openManipulatorOptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "print_open_manipulator_setting")
    open_manipulator_.printManipulatorSetting();
}

// void OpenManipulatorXController::displayPlannedPathCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
// {
//   trajectory_msgs::JointTrajectory joint_trajectory_planned = msg->trajectory[0].joint_trajectory;
//   joint_trajectory_ = joint_trajectory_planned;

//   if(moveit_plan_only_ == false)
//   {
//     log::println("[INFO] [OpenManipulator Controller] Execute Moveit planned path", "GREEN");
//     moveit_plan_state_ = true;
//   }
//   else
//     log::println("[INFO] [OpenManipulator Controller] Get Moveit planned path", "GREEN");
// }

// void OpenManipulatorXController::moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg)
// {
//   log::println("[INFO] [OpenManipulator Controller] Get Moveit plnning option", "GREEN");
//   moveit_plan_only_ = msg->goal.planning_options.plan_only; // click "plan & execute" or "plan" button

// }
// void OpenManipulatorXController::executeTrajGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg)
// {
//   log::println("[INFO] [OpenManipulator Controller] Execute Moveit planned path", "GREEN");
//   moveit_plan_state_ = true;
// }

/*****************************************************************************
** Callback Functions for ROS Servers
*****************************************************************************/
void OpenManipulatorXController::goalJointSpacePathCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    target_angle.push_back(req->joint_position.position.at(i));

  open_manipulator_.makeJointTrajectory(target_angle, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalJointSpacePathToKinematicsPoseCallback(
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

  open_manipulator_.makeJointTrajectory(req->end_effector_name, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalJointSpacePathToKinematicsPositionCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_.makeJointTrajectory(req->end_effector_name, target_pose.position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalJointSpacePathToKinematicsOrientationCallback(
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

  open_manipulator_.makeJointTrajectory(req->end_effector_name, target_pose.orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathCallback(
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
  open_manipulator_.makeTaskTrajectory(req->end_effector_name, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathPositionOnlyCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Vector3d position;
  position[0] = req->kinematics_pose.pose.position.x;
  position[1] = req->kinematics_pose.pose.position.y;
  position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectory(req->end_effector_name, position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathOrientationOnlyCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectory(req->end_effector_name, orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalJointSpacePathFromPresentCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    target_angle.push_back(req->joint_position.position.at(i));

  open_manipulator_.makeJointTrajectoryFromPresentPosition(target_angle, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathFromPresentCallback(
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

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req->planning_group, target_pose, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathFromPresentPositionOnlyCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Vector3d position;
  position[0] = req->kinematics_pose.pose.position.x;
  position[1] = req->kinematics_pose.pose.position.y;
  position[2] = req->kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req->planning_group, position, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalTaskSpacePathFromPresentOrientationOnlyCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req->planning_group, orientation, req->path_time);

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalToolControlCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  for(int i = 0; i < req->joint_position.joint_name.size(); i ++)
    open_manipulator_.makeToolTrajectory(req->joint_position.joint_name.at(i), req->joint_position.position.at(i));

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::setActuatorStateCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res)
{
  if(req->set_actuator_state == true) // enable actuators
  {
    log::println("Wait a second for actuator enable", "GREEN");
    open_manipulator_.enableAllActuator();
  }
  else // disable actuators
  {
    log::println("Wait a second for actuator disable", "GREEN");
    open_manipulator_.disableAllActuator();
  }

  res->is_planned = true;
  return;
}

void OpenManipulatorXController::goalDrawingTrajectoryCallback(
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

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req->end_effector_name, p_draw_circle_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req->param[0]; // x axis (m)
      draw_line_arg.kinematic.position(1) = req->param[1]; // y axis (m)
      draw_line_arg.kinematic.position(2) = req->param[2]; // z axis (m)
      void *p_draw_line_arg = &draw_line_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req->end_effector_name, p_draw_line_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req->param[0];  // radius (m)
      draw_rhombus_arg[1] = req->param[1];  // revolution (rev)
      draw_rhombus_arg[2] = req->param[2];  // start angle position (rad)
      void* p_draw_rhombus_arg = &draw_rhombus_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req->end_effector_name, p_draw_rhombus_arg, req->path_time);
    }
    else if(req->drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req->param[0];  // radius (m)
      draw_heart_arg[1] = req->param[1];  // revolution (rev)
      draw_heart_arg[2] = req->param[2];  // start angle position (rad)
      void* p_draw_heart_arg = &draw_heart_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req->end_effector_name, p_draw_heart_arg, req->path_time);
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

// void OpenManipulatorXController::getJointPositionMsgCallback(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Request>  req,
//   const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Response> res)
// {
//   rclcpp::AsyncSpinner spinner(1);
//   spinner.start();

//   const std::vector<std::string> &joint_names = move_group_->getJointNames();
//   std::vector<double> joint_values = move_group_->getCurrentJointValues();

//   for (std::size_t i = 0; i < joint_names.size(); i++)
//   {
//     res.joint_position.joint_name.push_back(joint_names[i]);
//     res.joint_position.position.push_back(joint_values[i]);
//   }

//   spinner.stop();
//   return;
// }

// void OpenManipulatorXController::getKinematicsPoseMsgCallback(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Request>  req,
//   const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Response> res)
// {
//   rclcpp::AsyncSpinner spinner(1);
//   spinner.start();

//   geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

//   res->header                     = current_pose.header;
//   res->kinematics_pose.pose       = current_pose.pose;

//   spinner.stop();
//   return;
// }

// void OpenManipulatorXController::setJointPositionMsgCallback(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
//   const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
// {
//   open_manipulator_msgs::msg::JointPosition msg = req->joint_position;
//   res->is_planned = calcPlannedPath(req->planning_group, msg);

//   return;
// }

// void OpenManipulatorXController::setKinematicsPoseMsgCallback(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
//   const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
// {
//   open_manipulator_msgs::msg::KinematicsPose msg = req->kinematics_pose;
//   res->is_planned = calcPlannedPath(req->planning_group, msg);

//   return;
// }

bool OpenManipulatorXController::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::msg::KinematicsPose msg)
{
  // rclcpp::AsyncSpinner spinner(1);
  // spinner.start();

  bool is_planned = false;
  geometry_msgs::msg::Pose target_pose = msg.pose;

  // move_group_->setPoseTarget(target_pose);

  // move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  // move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  // move_group_->setGoalTolerance(msg.tolerance);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (open_manipulator_.getMovingState() == false)
  {
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //   is_planned = true;
    // }
    // else
    {
      log::warn("Failed to Plan (task space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("Robot is Moving");
    is_planned = false;
  }

  // spinner.stop();
  return is_planned;
}

// bool OpenManipulatorXController::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::msg::JointPosition msg)
// {
//   // rclcpp::AsyncSpinner spinner(1);
//   // spinner.start();

//   bool is_planned = false;

//   // const robot_state::JointModelGroup *joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group);

//   // moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

//   // std::vector<double> joint_group_positions;
//   // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//   for (uint8_t index = 0; index < msg.position.size(); index++)
//   {
//     // joint_group_positions[index] = msg.position[index];
//   }

//   // move_group_->setJointValueTarget(joint_group_positions);

//   // move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
//   // move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   if (open_manipulator_.getMovingState() == false)
//   {
//     // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//     // if (success)
//     // {
//     //   is_planned = true;
//     // }
//     // else
//     {
//       log::warn("Failed to Plan (joint space goal)");
//       is_planned = false;
//     }
//   }
//   else
//   {
//     log::warn("Robot is moving");
//     is_planned = false;
//   }

//   // spinner.stop();
//   return is_planned;
// }

/********************************************************************************
** Functions related to processCallback 
********************************************************************************/
void OpenManipulatorXController::processCallback()   
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time present_time = clock.now();
  this->process(present_time.seconds());
}

void OpenManipulatorXController::process(double time)
{
  // moveitTimer(time);
  open_manipulator_.processOpenManipulator(time);
}

// void OpenManipulatorXController::moveitTimer(double present_time)
// {
//   static double priv_time = 0.0f;
//   static uint32_t step_cnt = 0;

//   if (moveit_plan_state_ == true)
//   {
//     double path_time = present_time - priv_time;
//     if (path_time > moveit_sampling_time_)
//     {
//       JointWaypoint target;
//       uint32_t all_time_steps = joint_trajectory_.points.size();

//       for(uint8_t i = 0; i < joint_trajectory_.points[step_cnt].positions.size(); i++)
//       {
//         JointValue temp;
//         temp.position = joint_trajectory_.points[step_cnt].positions.at(i);
//         temp.velocity = joint_trajectory_.points[step_cnt].velocities.at(i);
//         temp.acceleration = joint_trajectory_.points[step_cnt].accelerations.at(i);
//         target.push_back(temp);
//       }
//       open_manipulator_.makeJointTrajectory(target, path_time);

//       step_cnt++;
//       priv_time = present_time;

//       if (step_cnt >= all_time_steps)
//       {
//         step_cnt = 0;
//         moveit_plan_state_ = false;
//         if (moveit_update_start_state_pub_.getNumSubscribers() == 0)
//         {
//           log::warn("Could not update the start state! Enable External Communications at the Moveit Plugin");
//         }
//         std_msgs::Empty msg;
//         moveit_update_start_state_pub_->publish(msg);
//       }
//     }
//   }
//   else
//   {
//     priv_time = present_time;
//   }
// }

/********************************************************************************
** Functions related to publishCallback 
********************************************************************************/
void OpenManipulatorXController::publishCallback()   
{
  if (using_platform_ == true)  publishJointStates();
  // else  publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
}

void OpenManipulatorXController::publishOpenManipulatorStates()
{
  open_manipulator_msgs::msg::OpenManipulatorState msg;
  if(open_manipulator_.getMovingState())
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  else
    msg.open_manipulator_moving_state = msg.STOPPED;

  if(open_manipulator_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  open_manipulator_states_pub_->publish(msg);
}

void OpenManipulatorXController::publishKinematicsPose()
{
  open_manipulator_msgs::msg::KinematicsPose msg;
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:om_tools_name)
  {
    KinematicPose pose = open_manipulator_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_kinematics_pose_pub_.at(index)->publish(msg);
    index++;
  }
}

void OpenManipulatorXController::publishJointStates()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = rclcpp::Clock().now();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

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
  open_manipulator_joint_states_pub_->publish(msg);
}

void OpenManipulatorXController::publishGazeboCommand()
{
  JointWaypoint joint_value = open_manipulator_.getAllActiveJointValue();
  JointWaypoint tool_value = open_manipulator_.getAllToolValue();

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

  rclcpp::spin(std::make_unique<open_manipulator_x_controller::OpenManipulatorXController>(usb_port, baud_rate));

  rclcpp::shutdown();

  return 0;
}
