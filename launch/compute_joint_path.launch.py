# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa
"""
Starts the compute_joint_path action server, the compute_joint_trajectory time parametrization
server (package time_parametrization) and, optionally, the ik_solver node used by compute_joint_path
and the free-motion planner chain (collision_checker_server + parallel_multigoal_planner).

The robot (or its fake hardware) and move_group must already be running: the server reads
/robot_description, /joint_states and the planning scene from move_group.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def _time_parametrization_node(context):
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
        name='compute_joint_trajectory',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    pkg = get_package_share_directory('surface_path_planning')
    config_dir = os.path.join(pkg, 'config')

    args = [
        DeclareLaunchArgument('params', default_value=os.path.join(config_dir, 'compute_joint_path.yaml'),
                              description='ROS parameters of the action server'),
        DeclareLaunchArgument('time_parametrization_params',
                              default_value=os.path.join(config_dir, 'compute_joint_trajectory.yaml'),
                              description='ROS parameters of the time parametrization server'),
        DeclareLaunchArgument('joint_limits', default_value='',
                              description='MoveIt joint_limits.yaml of the robot used by the time parametrization '
                                          '(empty: URDF limits, accelerations = 5 * max velocity)'),
        DeclareLaunchArgument('mirrt_star_params', default_value=os.path.join(config_dir, 'mirrt_star.yaml'),
                              description='cnr_param file of the mirrt* solver of parallel_multigoal_planner'),
        DeclareLaunchArgument('ik_config', default_value=os.path.join(config_dir, 'compute_joint_path_ik.yaml'),
                              description='ik_solver configuration file'),
        DeclareLaunchArgument('start_ik_solver', default_value='true',
                              description='Start the ik_solver node (false if it is already running)'),
        DeclareLaunchArgument('start_multigoal_planner', default_value='true',
                              description='Start collision_checker_server and parallel_multigoal_planner '
                                          '(false if they are already running)'),
        DeclareLaunchArgument('ik_solver_ns', default_value='ur_ik_solver',
                              description='ik_solver namespace used by parallel_multigoal_planner (no leading /)'),
        DeclareLaunchArgument('collision_checker_params',
                              default_value=os.path.join(get_package_share_directory('collision_checker_server'),
                                                         'config', 'collision_checker_params.yaml'),
                              description='Parameters of collision_checker_server (group_name, padding, scaling)'),
    ]

    mirrt_star_params = ExecuteProcess(
        cmd=[FindExecutable(name='cnr_param_server'), '--path-to-file', LaunchConfiguration('mirrt_star_params')],
        output='screen',
    )

    ik_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ik_solver'), 'launch', 'ik_solver.launch.py')),
        launch_arguments={'config': LaunchConfiguration('ik_config')}.items(),
        condition=IfCondition(LaunchConfiguration('start_ik_solver')),
    )

    collision_checker = Node(
        package='collision_checker_server',
        executable='ik_collision_checker',
        name='ik_collision_check',
        output='screen',
        parameters=[LaunchConfiguration('collision_checker_params')],
        condition=IfCondition(LaunchConfiguration('start_multigoal_planner')),
    )

    multigoal_planner = Node(
        package='parallel_multigoal_planner',
        executable='parallel_multigoal_planner_node',
        name='parallel_multigoal_planner',
        output='screen',
        parameters=[{'ik_solver_ns': LaunchConfiguration('ik_solver_ns'),
                     'collision_check_service': '/collision_check'}],
        condition=IfCondition(LaunchConfiguration('start_multigoal_planner')),
    )

    server = Node(
        package='surface_path_planning',
        executable='compute_joint_path_node',
        name='compute_joint_path',
        output='screen',
        parameters=[LaunchConfiguration('params')],
    )

    # cnr_param_server instances must not run concurrently (they share the parameter directory):
    # the ik_solver parameters are loaded after the mirrt* ones, then the server starts.
    return LaunchDescription([
        *args,
        OpaqueFunction(function=_time_parametrization_node),
        mirrt_star_params,
        RegisterEventHandler(OnProcessExit(
            target_action=mirrt_star_params,
            on_exit=[ik_solver, collision_checker, multigoal_planner, TimerAction(period=3.0, actions=[server])],
        )),
    ])
