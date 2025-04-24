# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    PythonExpression,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
from launch.actions import OpaqueFunction

# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):

    mecanum = LaunchConfiguration('mecanum').perform(context)

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('x3plus_description'), 'urdf', 'yahboomcar_X3plusX.urdf.xacro')

    #robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = xacro.process_file(xacro_file, 
            mappings={  
                "mecanum": mecanum
                }).toxml()

    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
        output='both',
      parameters=[{
        'robot_description': robot_description_config
      }])

    return [robot_state_publisher_node]

# Configure ROS nodes for launch
def generate_launch_description():

    # args that can be set from the command line or a default will be used
    mecanum_launch_arg = DeclareLaunchArgument(
        "mecanum", default_value="True", description="Flag indicating to use mecanum wheels; skid drive otherwise"
    )
    mecanum_launch_value = LaunchConfiguration('mecanum')
    namespace = ''

    # Setup project paths
    pkg_project_gazebo = get_package_share_directory('x3plus_gazebo')
    pkg_project_description = get_package_share_directory('x3plus_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty.sdf'
        ])}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "master3_drive",
            "-allow_renaming",
            "false",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x", default="0.00"),
            "-y",
            LaunchConfiguration("y", default="0.00"),
            "-z",
            LaunchConfiguration("z", default="0.00"),
            "-R",
            LaunchConfiguration("roll", default="0.00"),
            "-P",
            LaunchConfiguration("pitch", default="0.00"),
            "-Y",
            LaunchConfiguration("yaw", default="0.00"),
        ],
        output="screen",
        namespace="",
    )


    # gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui')
    
    # # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_gazebo, 'config', 'x3plus_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_gazebo, 'config', 'x3plus_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        mecanum_launch_arg,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value=TextSubstitution(text="true"),
                               description='Open RViz.'),
        gz_spawn_entity,
        bridge,
        # joint_state_publisher_gui_node,
        # joint_state_publisher_node,
        # add OpaqueFunction to evaluate xacro file in context and pass to any nodes that need it
        OpaqueFunction(function=evaluate_xacro),
        rviz
    ])
