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

/* Authors: Ryan Shim, Hye-Jong KIM, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_X_CONTROLLER_H
#define OPEN_MANIPULATOR_X_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <boost/thread.hpp>
#include <unistd.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/planning_interface/planning_interface.h>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
// #include <moveit_msgs/MoveGroupActionGoal.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_drawing_trajectory.hpp"
#include "open_manipulator_msgs/srv/set_actuator_state.hpp"
#include "open_manipulator_msgs/srv/get_joint_position.hpp"
#include "open_manipulator_msgs/srv/get_kinematics_pose.hpp"
#include "open_manipulator_msgs/msg/open_manipulator_state.hpp"

#include "open_manipulator_x_libs/open_manipulator_x.h"

namespace open_manipulator_x_controller
{

class OpenManipulatorXController : public rclcpp::Node
{

 public:
  OpenManipulatorXController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorXController();

  // void publishCallback(const ros::TimerEvent&);  // any alternative...??????
  void publishCallback();  // any alternative...??????
  double getControlPeriod(void){return control_period_;}

  void startTimerThread();
  static void *timerThread(void *param);

  // void moveitTimer(double present_time);
  void process(double time);

  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::msg::KinematicsPose msg);
  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::msg::JointPosition msg);

 private:
  // ROS Parameters
  bool using_platform_;
  // bool using_moveit_;
  double control_period_;

  // flag parameter
  bool tool_ctrl_state_;
  bool timer_thread_state_;
  // bool moveit_plan_state_;

  // // MoveIt! interface
  // moveit::planning_interface::MoveGroupInterface* move_group_;
  // trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  // double moveit_sampling_time_;
  // bool moveit_plan_only_;

  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;

  // Related robotis_manipulator
  OpenManipulator open_manipulator_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Publisher<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr open_manipulator_states_pub_;
  std::vector<rclcpp::Publisher<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr> open_manipulator_kinematics_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr open_manipulator_joint_states_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gazebo_goal_joint_position_pub_;
  // rclcpp::Publisher<????>::SharedPtr moveit_update_start_state_pub_;

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr open_manipulator_option_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::MoveGroupActionGoal>::SharedPtr move_group_goal_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::ExecuteTrajectoryActionGoal>::SharedPtr execute_traj_goal_sub_;

  void openManipulatorOptionCallback(const std_msgs::msg::String::SharedPtr msg);
  // void displayPlannedPathCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
  // void moveGroupGoalCallback(const moveit_msgs::msg::MoveGroupActionGoal::SharedPtr msg);
  // void executeTrajGoalCallback(const moveit_msgs::msg::ExecuteTrajectoryActionGoal::SharedPtr msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_to_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_to_kinematics_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_to_kinematics_orientation_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_from_present_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_from_present_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_task_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_actuator_state_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_drawing_trajectory_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr get_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr get_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_kinematics_pose_server_;

  void goalJointSpacePathCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goalJointSpacePathToKinematicsPoseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalJointSpacePathToKinematicsPositionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalJointSpacePathToKinematicsOrientationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalTaskSpacePathCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalTaskSpacePathPositionOnlyCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalTaskSpacePathOrientationOnlyCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalJointSpacePathFromPresentCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goalTaskSpacePathFromPresentCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalTaskSpacePathFromPresentPositionOnlyCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalTaskSpacePathFromPresentOrientationOnlyCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goalToolControlCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void setActuatorStateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res);
  void goalDrawingTrajectoryCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res);
  void setJointPositionMsgCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void setKinematicsPoseMsgCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void getJointPositionMsgCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Response> res);
  void getKinematicsPoseMsgCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Response> res);
};

}  // namespace open_manipulator_x_controller

#endif //OPEN_MANIPULATOR_X_CONTROLLER_H
