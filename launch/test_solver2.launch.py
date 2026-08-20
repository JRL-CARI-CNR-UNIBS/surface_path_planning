from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import os

def generate_launch_description():
    surface_path_planning_dir = get_package_share_directory('surface_path_planning')

    # --- Launch configurations ---
    test_mode = LaunchConfiguration('test_mode') 
    use_sharework = LaunchConfiguration('use_sharework', default='true')

    # --- Launch parameters ---
    test_solver_config_file_arg = DeclareLaunchArgument(
        'test_solver_param_file',
        default_value='test_solver.yaml',
        description='Path to the test solver parameter file'
    )

    test_solver_test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='0',
        description='Set to an int N>0 to enable testing mode (no action client), and repeat the test N times'
    )

    use_sharework_arg = DeclareLaunchArgument(
        'use_sharework',
        default_value='true',
        description='Set to true to use the Sharework cell robot configuration'
    )

    # --- Param servers ---
    test_solver_param_file_path = PathJoinSubstitution([
        surface_path_planning_dir,
        'config',
        'test_solver.yaml'
    ])
    test_solver_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', test_solver_param_file_path],
        output='screen'
    )

    ik_param_file_path = PathJoinSubstitution([
        surface_path_planning_dir,
        'config',
        'ik_params.yaml'
    ])

    ik_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', ik_param_file_path],
        output='screen'
    )

    #constraint_visualizer = Node(
    #    package='surface_path_planning',
    #    executable='geometric_constraints_visualizer.py',
    #    output='screen',
    #    parameters=[{'constaints_description': test_solver_param_file_path}]
    #)

    planner = Node(
        package='surface_path_planning',
        executable='simple_test_2',
        output='screen',
        parameters=[{'test_mode': LaunchConfiguration('test_mode'), 
                     'use_sharework': LaunchConfiguration('use_sharework')}]
    )

    # --- Add delays between launches ---
    return LaunchDescription([
        test_solver_config_file_arg,
        test_solver_test_mode_arg,
        use_sharework_arg,
        test_solver_params,
        ik_params,
        #constraint_visualizer,
        TimerAction(period=3.0, actions=[planner]),
    ])
