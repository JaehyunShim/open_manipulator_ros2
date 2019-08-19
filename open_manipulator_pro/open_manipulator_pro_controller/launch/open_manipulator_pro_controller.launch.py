# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Jaehyun Shim */

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_robot_name         = LaunchConfiguration('use_robot_name',         default='open_manipulator_pro')
    dynamixel_usb_port     = LaunchConfiguration('dynamixel_usb_port',     default='/dev/ttyUSB0')
    dynamixel_baud_rate    = LaunchConfiguration('dynamixel_baud_rate',    default='1000000')
    control_period         = LaunchConfiguration('control_period',         default='0.010')
    use_platform           = LaunchConfiguration('use_platform',           default='True')
    use_moveit             = LaunchConfiguration('use_moveit',             default='False')
    planning_group_name    = LaunchConfiguration('planning_group_name',    default='arm')
    moveit_sample_duration = LaunchConfiguration('moveit_sample_duration', default='0.050') 

    # param_file_name = 'open_manipulator_pro_controller_params.yaml'
    # param_path = os.path.join(
    #     get_package_share_directory('open_manipulator_pro_controller'), 
    #     'param', param_file_name
    # )

    param_path = '/home/robotis/colcon_ws/src/ROS2_study/open_manipulator_pro/open_manipulator_pro_controller/param/open_manipulator_pro_controller_params.yaml'

#    print(param_path)
        
    return LaunchDescription([
        Node(
            package='open_manipulator_pro_controller',
            node_executable='open_manipulator_pro_controller',
            node_name='open_manipulator_pro_controller',
            # arguments=['-d', usb_port, baud_rate],
            parameters=[{'use_platform': use_platform}],
            output='screen')
    ])
