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

#ifndef OPEN_MANIPULATOR_PRO_CONTROLLER_HPP
#define OPEN_MANIPULATOR_PRO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <memory>

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

#include "open_manipulator_pro_libs/open_manipulator_pro.hpp"

namespace open_manipulator_pro_controller
{

class OpenManipulatorProController : public rclcpp::Node
{
 public:
  OpenManipulatorProController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorProController();

  void process_callback(); 
  void publish_callback();  
  double get_control_period(void){return control_period_;}

  // void moveit_timer(double present_time);
  void process(double time);

  bool calc_planned_path(const std::string planning_group, open_manipulator_msgs::msg::KinematicsPose msg);
  bool calc_planned_path(const std::string planning_group, open_manipulator_msgs::msg::JointPosition msg);

  rclcpp::TimerBase::SharedPtr process_timer;
  rclcpp::TimerBase::SharedPtr publish_timer;

 private:
  /*****************************************************************************
  ** Parameters
  *****************************************************************************/
  bool using_platform_;
  bool with_gripper_;
  double control_period_;
  // bool using_moveit_;
  // bool moveit_plan_state_;

  // // MoveIt! interface
  // moveit::planning_interface::MoveGroupInterface* move_group_;
  // trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  double moveit_sampling_time_;
  // bool moveit_plan_only_;

  // Robotis_manipulator related 
  OpenManipulatorPro open_manipulator_pro_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void init_publisher();
  void init_subscriber();
  void init_server();

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Publisher<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr open_manipulator_pro_states_pub_;
  std::vector<rclcpp::Publisher<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr> open_manipulator_pro_kinematics_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr open_manipulator_pro_joint_states_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gazebo_goal_joint_position_pub_;
  // rclcpp::Publisher<????>::SharedPtr moveit_update_start_state_pub_;

  void publish_open_manipulator_pro_states();
  void publish_kinematics_pose();
  void publish_joint_states();
  void publish_gazebo_command();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr open_manipulator_pro_option_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::MoveGroupActionGoal>::SharedPtr move_group_goal_sub_;
  // rclcpp::Subscription<moveit_msgs::msg::ExecuteTrajectoryActionGoal>::SharedPtr execute_traj_goal_sub_;

  void open_manipulator_pro_option_callback(const std_msgs::msg::String::SharedPtr msg);
  // void displayPlannedPathCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
  // void moveGroupGoalCallback(const moveit_msgs::msg::MoveGroupActionGoal::SharedPtr msg);
  // void executeTrajGoalCallback(const moveit_msgs::msg::ExecuteTrajectoryActionGoal::SharedPtr msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_orientation_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetActuatorState>::SharedPtr set_actuator_state_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetDrawingTrajectory>::SharedPtr goal_drawing_trajectory_server_;
  rclcpp::Service<open_manipulator_msgs::srv::GetJointPosition>::SharedPtr get_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::GetKinematicsPose>::SharedPtr get_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr set_kinematics_pose_server_;

  void goal_joint_space_path_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goal_joint_space_path_to_kinematics_pose_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_to_kinematics_position_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_to_kinematics_orientation_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_position_only_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_orientation_only_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_from_present_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goal_task_space_path_from_present_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_from_present_position_only_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_from_present_orientation_only_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_tool_control_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void set_actuator_state_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res);
  void goal_drawing_trajectory_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res);
  void get_joint_position_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Response> res);
  void get_kinematics_pose_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Response> res);
  void set_joint_position_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void set_kinematics_pose_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
};
}  // namespace open_manipulator_pro_controller
#endif //OPEN_MANIPULATOR_PRO_CONTROLLER_HPP
