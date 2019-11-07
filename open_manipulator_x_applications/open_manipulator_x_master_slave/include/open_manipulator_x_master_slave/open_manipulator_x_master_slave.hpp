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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP
#define OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP

#include <termios.h>
#include <sys/ioctl.h>

#include <rclcpp/rclcpp.hpp>

#include "open_manipulator_libs/dynamixel.h"
#include "open_manipulator_libs/kinematics.h"
#include "open_manipulator_msgs/SetJointPosition.h"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL  "tool_dxl"
#define NUM_OF_JOINT 4

#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)

#define MASTER_SLAVE_MODE 0
#define START_RECORDING_TRAJECTORY_MODE 1
#define STOP_RECORDING_TRAJECTORY_MODE 2
#define PLAY_RECORDED_TRAJECTORY_MODE 3

using namespace robotis_manipulator;

typedef struct _WaypointBuffer
{
  std::vector<double> joint_angle;
  double tool_position;
} WaypointBuffer;


class OpenManipulatorXMasterSlave : public RobotisManipulator
{
 public:
  OpenManipulatorXMasterSlave(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorXMasterSlave();

  void publish_callback();  
  
  rclcpp::TimerBase::SharedPtr publish_timer_;

 private:
  /*****************************************************************************
  ** Parameters
  *****************************************************************************/
  uint8_t mode_state_;
  double service_call_period_;
  std::vector<uint8_t> dxl_id_;
  std::vector<double> goal_joint_position_;
  double goal_tool_position_;

  std::vector<WaypointBuffer> record_buffer_;
  int buffer_index_;

  JointActuator *actuator_;
  ToolActuator *tool_;

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  void init_open_manipulator_x(STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", double service_call_period = 0.010);
  void sync_open_manipulator_x(bool recorded_state);
  double get_service_call_period() {return service_call_period_;}

  bool set_joint_space_path(double path_time, std::vector<double> set_goal_joint_position = {});
  bool set_tool_path(double set_goal_tool_position = -1.0);
  void set_mode_state(char ch);

  void print_text();
  bool kbhit();
};
#endif //OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP
