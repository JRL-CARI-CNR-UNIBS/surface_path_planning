# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa
"""
Starts the compute_joint_path and compute_joint_trajectory action servers (one node), the
apply_time_parametrization server (package time_parametrization, used by compute_joint_trajectory)
and, optionally, the ik_solver node and the free-motion planner selected by multigoal_planner:
  - parallel_multigoal_planner (default): collision_checker_server + parallel_multigoal_planner;
  - prm_multigoal_planner: prm_multigoal_planner (collision_checker_server is started too, it is only
    used to explain failed requests).

The robot (or its fake hardware) and move_group must already be running: the server reads
/robot_description, /joint_states and the planning scene from move_group.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def _time_parametrization_node(context):
    if LaunchConfiguration('start_time_parametrization').perform(context).lower() not in ('true', '1'):
        return []
    # The server builds its RobotModel from the URDF only: without the MoveIt joint limits
    # (robot_description_planning) the acceleration limits default to 5 * max velocity.
    parameters = [LaunchConfiguration('time_parametrization_params')]
    joint_limits_file = LaunchConfiguration('joint_limits').perform(context)
    if joint_limits_file:
        with open(joint_limits_file) as f:
            parameters.append({'robot_description_planning': yaml.safe_load(f)})
    return [Node(
        package='time_parametrization',
        executable='apply_time_parametrization_server',
        name='apply_time_parametrization',
        output='screen',
        parameters=parameters,
    )]


PLANNERS = ('parallel_multigoal_planner', 'prm_multigoal_planner')


def _planner_nodes(context):
    planner = LaunchConfiguration('multigoal_planner').perform(context)
    if planner not in PLANNERS:
        raise RuntimeError(f"multigoal_planner must be one of {PLANNERS}, got '{planner}'")

    server = Node(
        package='surface_path_planning',
        executable='compute_joint_path_node',
        name='compute_joint_path',
        output='screen',
        parameters=[LaunchConfiguration('params'), {'multigoal_planner': planner}],
    )
    nodes = [TimerAction(period=3.0, actions=[server])]
    if LaunchConfiguration('start_multigoal_planner').perform(context).lower() not in ('true', '1'):
        return nodes

    nodes.append(Node(
        package='collision_checker_server',
        executable='ik_collision_checker',
        name='ik_collision_check',
        output='screen',
        parameters=[LaunchConfiguration('collision_checker_params')],
    ))
    if planner == 'parallel_multigoal_planner':
        nodes.append(Node(
            package='parallel_multigoal_planner',
            executable='parallel_multigoal_planner_node',
            name='parallel_multigoal_planner',
            output='screen',
            parameters=[LaunchConfiguration('mirrt_star_params'),
                        {'ik_solver_ns': LaunchConfiguration('ik_solver_ns'),
                         'collision_check_service': '/collision_check'}],
        ))
    else:
        nodes.append(Node(
            package='prm_multigoal_planner',
            executable='prm_multigoal_planner_node',
            name='prm_multigoal_planner',
            output='screen',
            parameters=[LaunchConfiguration('prm_params'),
                        {'ik_solver_ns': LaunchConfiguration('ik_solver_ns'),
                         'action_name': 'prm_multigoal_plan'}],
        ))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('surface_path_planning')
    config_dir = os.path.join(pkg, 'config')

    args = [
        DeclareLaunchArgument('params', default_value=os.path.join(config_dir, 'compute_joint_path.yaml'),
                              description='ROS parameters of the action server'),
        DeclareLaunchArgument('time_parametrization_params',
                              default_value=os.path.join(config_dir, 'apply_time_parametrization.yaml'),
                              description='ROS parameters of the time parametrization server'),
        DeclareLaunchArgument('start_time_parametrization', default_value='true',
                              description='Start apply_time_parametrization (false if it is already running, '
                                          'e.g. started by picking_example)'),
        DeclareLaunchArgument('joint_limits', default_value='',
                              description='MoveIt joint_limits.yaml of the robot used by the time parametrization '
                                          '(empty: URDF limits, accelerations = 5 * max velocity)'),
        DeclareLaunchArgument('multigoal_planner', default_value='parallel_multigoal_planner',
                              description='Free-motion planner: parallel_multigoal_planner or prm_multigoal_planner'),
        DeclareLaunchArgument('prm_params',
                              default_value=os.path.join(get_package_share_directory('prm_multigoal_planner'),
                                                         'config', 'prm_multigoal_planner.yaml'),
                              description='ROS 2 parameter file of prm_multigoal_planner'),
        DeclareLaunchArgument('mirrt_star_params', default_value=os.path.join(config_dir, 'mirrt_star.yaml'),
                              description='ROS 2 parameter file of the mirrt* solver of parallel_multigoal_planner'),
        DeclareLaunchArgument('ik_config', default_value=os.path.join(config_dir, 'compute_joint_path_ik.yaml'),
                              description='ik_solver configuration file'),
        DeclareLaunchArgument('start_ik_solver', default_value='true',
                              description='Start the ik_solver node (false if it is already running)'),
        DeclareLaunchArgument('start_multigoal_planner', default_value='true',
                              description='Start collision_checker_server and the selected multigoal planner '
                                          '(false if they are already running)'),
        DeclareLaunchArgument('ik_solver_ns', default_value='ur_ik_solver',
                              description='ik_solver namespace used by the multigoal planner (no leading /)'),
        DeclareLaunchArgument('collision_checker_params',
                              default_value=os.path.join(get_package_share_directory('collision_checker_server'),
                                                         'config', 'collision_checker_params.yaml'),
                              description='Parameters of collision_checker_server (group_name, padding, scaling)'),
    ]

    ik_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ik_solver'), 'launch', 'ik_solver.launch.py')),
        launch_arguments={'config': LaunchConfiguration('ik_config')}.items(),
        condition=IfCondition(LaunchConfiguration('start_ik_solver')),
    )

    return LaunchDescription([
        *args,
        OpaqueFunction(function=_time_parametrization_node),
        ik_solver,
        OpaqueFunction(function=_planner_nodes),
    ])
