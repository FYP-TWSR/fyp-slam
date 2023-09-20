# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.




# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped







import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('simulation'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('simulation'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf','xacro',
                              'car.urdf.xacro')
    # gazebo_ros2_control_demos_path = os.path.join(
    #     get_package_share_directory('gazebo_ros2_control_demos'))
    
    # xacro_file = os.path.join(gazebo_ros2_control_demos_path,
    #                           'urdf',
    #                           'test_diff_drive.xacro.urdf')


    world_file_path = os.path.join(
        get_package_share_directory('simulation'))
    world_file = os.path.join(world_file_path, 'worlds', 'box_house.world')
    # load_world = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', world_file],
    #     output='screen'
    # )
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )
    # keyboard_control_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     output='screen',
    #     remappings=[('/diff_drive_base_controller/cmd_vel_unstamped','/cmd_vel')]
    # )
    

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diffbot'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='world_file',
            description='Specify world file name'
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        gazebo,
        # load_world,
        node_robot_state_publisher,
        spawn_entity,
    ])
