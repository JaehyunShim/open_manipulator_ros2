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

struct OpenManipulatorXTeleopJoystick::Impl
{

};

OpenManipulatorXTeleopJoystick::OpenManipulatorXTeleopJoystick()
: Node("open_manipulator_x_teleop_joystick")
{
  pimpl_ = new Impl;
  // pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
  //   std::bind(&OpenManipulatorXTeleopJoystick::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  // Initialise joint angle and kinematic position size 
  present_joint_angle.resize(NUM_OF_JOINT);
  present_kinematic_position.resize(3);

  // Initialise Subscribers
  joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&OpenManipulatorXTeleopJoystick::jointStatesCallback, this, std::placeholders::_1));
  kinematics_pose_sub = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "kinematics_pose", 10, std::bind(&OpenManipulatorXTeleopJoystick::kinematicsPoseCallback, this, std::placeholders::_1));
  joy_command_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&OpenManipulatorXTeleopJoystick::joyCallback2, this, std::placeholders::_1));

  // Initialise Clients
  goal_task_space_path_from_present_position_only_client = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_joint_space_path_client = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");

  RCLCPP_INFO(this->get_logger(), "OpenManipulator initialised");
  


  // enable_button = this->declare_parameter("enable_button", 5L);

  // enable_turbo_button = this->declare_parameter("enable_turbo_button", -1L);

  // std::map<std::string, int64_t> default_linear_map{
  //   {"x", 5L},
  //   {"y", -1L},
  //   {"z", -1L},
  // };
  // this->declare_parameters("axis_linear", default_linear_map);
  // this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  // std::map<std::string, int64_t> default_angular_map{
  //   {"yaw", 2L},
  //   {"pitch", -1L},
  //   {"roll", -1L},
  // };
  // this->declare_parameters("axis_angular", default_angular_map);
  // this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  // std::map<std::string, double> default_scale_linear_normal_map{
  //   {"x", 0.5},
  //   {"y", 0.0},
  //   {"z", 0.0},
  // };
  // this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  // this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  // std::map<std::string, double> default_scale_linear_turbo_map{
  //   {"x", 1.0},
  //   {"y", 0.0},
  //   {"z", 0.0},
  // };
  // this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  // this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  // std::map<std::string, double> default_scale_angular_normal_map{
  //   {"yaw", 0.5},
  //   {"pitch", 0.0},
  //   {"roll", 0.0},
  // };
  // this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  // this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  // std::map<std::string, double> default_scale_angular_turbo_map{
  //   {"yaw", 1.0},
  //   {"pitch", 0.0},
  //   {"roll", 0.0},
  // };
  // this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  // this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  // ROS_INFO_NAMED("OpenManipulatorXTeleopJoystick", "Teleop enable button %" PRId64 ".", pimpl_->enable_button);
  // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "OpenManipulatorXTeleopJoystick",
  //   "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);

  // for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
  //      it != pimpl_->axis_linear_map.end(); ++it)
  // {
  //   ROS_INFO_COND_NAMED(it->second != -1L, "OpenManipulatorXTeleopJoystick", "Linear axis %s on %" PRId64 " at scale %f.",
  //     it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
  //   ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "OpenManipulatorXTeleopJoystick",
  //     "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  // }

  // for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
  //      it != pimpl_->axis_angular_map.end(); ++it)
  // {
  //   ROS_INFO_COND_NAMED(it->second != -1L, "OpenManipulatorXTeleopJoystick", "Angular axis %s on %" PRId64 " at scale %f.",
  //     it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
  //   ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "OpenManipulatorXTeleopJoystick",
  //     "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  // }

  // pimpl_->sent_disable_msg = false;
}

OpenManipulatorXTeleopJoystick::~OpenManipulatorXTeleopJoystick()
{
  delete pimpl_;
}

// double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
//               const std::map<std::string, double>& scale_map, const std::string& fieldname)
// {
//   if (axis_map.find(fieldname) == axis_map.end() ||
//       axis_map.at(fieldname) == -1L ||
//       scale_map.find(fieldname) == scale_map.end() ||
//       static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
//   {
//     return 0.0;
//   }

//   return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
// }

// void OpenManipulatorXTeleopJoystick::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
//                                          const std::string& which_map)
// {
//   // Initializes with zeros by default.
//   auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

//   cmd_vel_msg->linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
//   cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
//   cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
//   cmd_vel_msg->angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
//   cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
//   cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

//   cmd_vel_pub->publish(std::move(cmd_vel_msg));
//   sent_disable_msg = false;
// }

// void OpenManipulatorXTeleopJoystick::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
// {
//   if (enable_turbo_button >= 0 &&
//       static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
//       joy_msg->buttons[enable_turbo_button])
//   {
//     sendCmdVelMsg(joy_msg, "turbo");
//   }
//   else if (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
//            joy_msg->buttons[enable_button])
//   {
//     sendCmdVelMsg(joy_msg, "normal");
//   }
//   else
//   {
//     // When enable button is released, immediately send a single no-motion command
//     // in order to stop the robot.
//     if (!sent_disable_msg)
//     {
//       // Initializes with zeros by default.
//       auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
//       cmd_vel_pub->publish(std::move(cmd_vel_msg));
//       sent_disable_msg = true;
//     }
//   }
// }



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
  present_joint_angle = temp_angle;
}

void OpenManipulatorXTeleopJoystick::kinematicsPoseCallback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position = temp_position;
}

void OpenManipulatorXTeleopJoystick::joyCallback2(const sensor_msgs::msg::Joy::SharedPtr msg)
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
  
  auto response_future = goal_joint_space_path_client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(this->shared_from_this(), response_future) ==
  //   rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
    // auto response = response_future.get();
    // return response->is_planned;
  // }
  // RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  // return false;
}

bool OpenManipulatorXTeleopJoystick::setToolControl(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  // request->joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  auto response_future = goal_tool_control_client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(this->shared_from_this(), response_future) ==
  //   rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
    // auto response = response_future.get();
    // return response->is_planned;
  // }
  // RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  // return false;
}

bool OpenManipulatorXTeleopJoystick::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  // request->planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  auto response_future = goal_task_space_path_from_present_position_only_client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(this->shared_from_this(), response_future) ==
  //   rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
    // auto response = response_future.get();
    // return response->is_planned;
  // }
  // RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  // return false;
}

}  // namespace open_manipulator_x_teleop_joystick


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<open_manipulator_x_teleop_joystick::OpenManipulatorXTeleopJoystick>());

  rclcpp::shutdown();

  return 0;
}
