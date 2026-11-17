#!/usr/bin/env python3
# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa
"""
Example client of the compute_joint_path and compute_joint_trajectory actions.

1. Calls the compute_trajectory service of surface_raster_planner (a mesh must already be loaded).
2. Sends the resulting PoseArray and jump_indices to the compute_joint_path action, which returns
   the joint path split into surface / free-motion paths.
3. Sends the paths to the compute_joint_trajectory action, which retimes each of them (TOTG) and
   appends them in one trajectory.
4. Executes the trajectory with MoveIt (execute_trajectory action of move_group).
The server publishes the path and then the trajectory for Rviz on /compute_joint_path/display_path.

ROS parameters:
  scaling_factor (double, default 0.1): velocity and acceleration scaling factor of the surface
    paths, in (0, 1]
  free_motion_scaling_factor (double, default: scaling_factor): the same for the free-motion paths
  execute (bool, default true): false to stop after the time parametrization

Example:
  ros2 run surface_path_planning send_raster_to_joint_path.py --planning-frame open_tip \
      --row-step 0.02 --deviation 0 0 0.05 --ros-args -p scaling_factor:=0.2
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from surface_path_planning_msgs.action import ComputeJointPath, ComputeJointTrajectory
from surface_raster_planner_msgs.srv import ComputeTrajectory


def call_action(node, client, goal, feedback_callback=None):
    """Send a goal and wait for the result. Returns (accepted, wrapped_result)."""
    goal_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, goal_future)
    handle = goal_future.result()
    if not handle.accepted:
        return False, None
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    return True, result_future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--raster-service', default='/raster_planner_node/compute_trajectory')
    parser.add_argument('--action', default='/compute_joint_path')
    parser.add_argument('--trajectory-action', default='/compute_joint_trajectory')
    parser.add_argument('--execute-action', default='/execute_trajectory')
    parser.add_argument('--row-step', type=float, default=0.02, help='[m] distance between raster rows')
    parser.add_argument('--keep-tool-orientation', action='store_true')
    parser.add_argument('--planning-frame', default='open_tip', help='robot frame that must follow the poses')
    parser.add_argument('--deviation', type=float, nargs=3, default=[0.0, 0.0, 0.05], metavar=('X', 'Y', 'Z'),
                        help='[m] leave/approach offset in the tool frame')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('send_raster_to_joint_path')
    scaling_factor = node.declare_parameter('scaling_factor', 0.1).value
    free_motion_scaling_factor = node.declare_parameter('free_motion_scaling_factor', scaling_factor).value
    execute = node.declare_parameter('execute', True).value
    for name, value in (('scaling_factor', scaling_factor), ('free_motion_scaling_factor', free_motion_scaling_factor)):
        if not 0.0 < value <= 1.0:
            node.get_logger().error(f'{name} must be in (0, 1], got {value}')
            return 1

    # 1) Raster path
    raster = node.create_client(ComputeTrajectory, args.raster_service)
    if not raster.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'service {args.raster_service} not available')
        return 1
    req = ComputeTrajectory.Request(row_step=args.row_step, keep_tool_orientation=args.keep_tool_orientation)
    future = raster.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    raster_res = future.result()
    if raster_res is None or not raster_res.success:
        node.get_logger().error(f'raster planner failed: {raster_res.message if raster_res else "no response"}')
        return 1
    node.get_logger().info(f'raster: {len(raster_res.poses.poses)} poses, jumps {list(raster_res.jump_indices)}')

    # 2) Joint path
    client = ActionClient(node, ComputeJointPath, args.action)
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f'action {args.action} not available')
        return 1

    goal = ComputeJointPath.Goal()
    goal.poses = raster_res.poses
    goal.jump_indices = raster_res.jump_indices
    goal.planning_frame = args.planning_frame
    goal.deviation_distance.x, goal.deviation_distance.y, goal.deviation_distance.z = args.deviation

    def on_feedback(msg):
        fb = msg.feedback
        node.get_logger().info(f'[{fb.current_index}/{fb.total_poses}] {fb.status}', throttle_duration_sec=1.0)

    accepted, wrapped = call_action(node, client, goal, on_feedback)
    if not accepted:
        node.get_logger().error('goal rejected (see the server log)')
        return 1
    result = wrapped.result

    if not result.success:
        node.get_logger().error(f'compute_joint_path failed: {result.message}')
        return 1
    reached = sum(1 for i in result.pose_indices if i >= 0)
    node.get_logger().info(f'{result.message}: {len(result.trajectory.points)} points, {len(result.paths)} paths, '
                           f'{reached} poses reached, skipped {list(result.skipped_indices)}')

    # 3) Time parametrization of the paths
    traj_client = ActionClient(node, ComputeJointTrajectory, args.trajectory_action)
    if not traj_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f'action {args.trajectory_action} not available')
        return 1
    traj_goal = ComputeJointTrajectory.Goal()
    traj_goal.paths = result.paths
    traj_goal.path_types = result.path_types
    traj_goal.surface_velocity_scaling = scaling_factor
    traj_goal.surface_acceleration_scaling = scaling_factor
    traj_goal.free_motion_velocity_scaling = free_motion_scaling_factor
    traj_goal.free_motion_acceleration_scaling = free_motion_scaling_factor
    accepted, wrapped = call_action(node, traj_client, traj_goal)
    if not accepted:
        node.get_logger().error('compute_joint_trajectory goal rejected (see the server log)')
        return 1
    if not wrapped.result.success:
        node.get_logger().error(f'compute_joint_trajectory failed: {wrapped.result.message}')
        return 1
    timed = wrapped.result.trajectory
    duration = timed.joint_trajectory.points[-1].time_from_start
    node.get_logger().info(f'trajectory: {len(timed.joint_trajectory.points)} points, '
                           f'{duration.sec + duration.nanosec * 1e-9:.2f} s')

    # 4) Execution with MoveIt
    if execute:
        exec_client = ActionClient(node, ExecuteTrajectory, args.execute_action)
        if not exec_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error(f'action {args.execute_action} not available')
            return 1
        accepted, wrapped = call_action(node, exec_client, ExecuteTrajectory.Goal(trajectory=timed))
        if not accepted:
            node.get_logger().error('execution goal rejected by move_group')
            return 1
        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            node.get_logger().error(f'execution failed (error code {code})')
            return 1
        node.get_logger().info('trajectory executed')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
