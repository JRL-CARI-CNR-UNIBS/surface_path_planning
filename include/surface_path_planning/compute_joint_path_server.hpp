// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ik_solver_msgs/srv/change_tool.hpp>
#include <ik_solver_msgs/srv/collision_checking.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
// MoveIt >= Jazzy ships .hpp headers, Humble only .h
#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
#include <moveit/planning_scene/planning_scene.hpp>
#else
#include <moveit/planning_scene/planning_scene.h>
#endif
// MoveIt >= Jazzy ships .hpp headers, Humble only .h
#if __has_include(<moveit/robot_model_loader/robot_model_loader.hpp>)
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#else
#include <moveit/robot_model_loader/robot_model_loader.h>
#endif
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <parallel_multigoal_planner_msgs/action/parallel_multigoal_plan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surface_path_planning_msgs/action/compute_joint_path.hpp>
#include <surface_path_planning_msgs/action/compute_joint_trajectory.hpp>
#include <time_parametrization_msgs/action/apply_time_parametrization.hpp>

#include "surface_path_planning/graph_core_ros.hpp"

#include "surface_path_planning/joint_path_builder.hpp"

namespace surface_path_planning
{

/**
 * @brief Action servers "compute_joint_path" (surface_path_planning_msgs/action/ComputeJointPath)
 * and "compute_joint_trajectory" (surface_path_planning_msgs/action/ComputeJointTrajectory).
 *
 * For each compute_joint_path goal:
 *  1. reads the robot configuration from the joint states topic;
 *  2. downloads the current planning scene from move_group (used for collision checking);
 *  3. sets goal.planning_frame as IK tool frame (ik_solver change_tool service);
 *  4. solves with one call of get_task_redundant_ik_array the IK of every pose and of every pose
 *     moved by goal.deviation_distance (in the tool frame);
 *  5. builds the joint path with JointPathBuilder; free motions are planned by the
 *     parallel_multigoal_planner or by the prm_multigoal_planner action (parameter multigoal_planner);
 *  6. splits the path into surface / free-motion paths (see splitPath());
 *  7. publishes the path for Rviz (moveit_msgs/DisplayTrajectory, fixed time step between points).
 *
 * For each compute_joint_trajectory goal, every path is retimed with its own request to the time
 * parametrization action (time_parametrization package) and the results are appended in one
 * trajectory, which is also published for Rviz.
 *
 * Each action accepts one goal at a time; the two actions can run at the same time.
 */
class ComputeJointPathServer
{
public:
  using Action = surface_path_planning_msgs::action::ComputeJointPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using TrajectoryAction = surface_path_planning_msgs::action::ComputeJointTrajectory;
  using TrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<TrajectoryAction>;

  /**
   * @brief Declare the parameters, create clients, subscriptions and the action server.
   * @param node Node; it must be spun by a multi-threaded executor.
   */
  explicit ComputeJointPathServer(const rclcpp::Node::SharedPtr& node);

  /// Loads the robot model. Call it before adding the node to an executor (the model loader spins the node).
  bool init();

private:
  /**
   * @brief Accept a goal only if no other goal is running.
   */
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Action::Goal> goal);
  /**
   * @brief Accept cancellation requests.
   */
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> gh);
  /**
   * @brief Run execute() in a separate thread.
   */
  void handleAccepted(const std::shared_ptr<GoalHandle> gh);
  /**
   * @brief Compute the joint path of one goal (steps listed in the class description).
   */
  void execute(const std::shared_ptr<GoalHandle> gh);

  /// compute_joint_trajectory: validate the goal and accept it only if no other one is running.
  rclcpp_action::GoalResponse handleTrajectoryGoal(const rclcpp_action::GoalUUID& uuid,
                                                   std::shared_ptr<const TrajectoryAction::Goal> goal);
  /// compute_joint_trajectory: run executeTrajectory() in a separate thread.
  void handleTrajectoryAccepted(const std::shared_ptr<TrajectoryGoalHandle> gh);
  /// compute_joint_trajectory: retime the paths of one goal and append them.
  void executeTrajectory(const std::shared_ptr<TrajectoryGoalHandle> gh);

  /**
   * @brief Read the current robot configuration from the joint states topic.
   */
  bool getCurrentConfiguration(Eigen::VectorXd& q, std::string& error);
  /**
   * @brief Download the planning scene from move_group.
   */
  bool updatePlanningScene(std::string& error);
  /**
   * @brief Set the IK tool frame with the ik_solver change_tool service.
   */
  bool changeTool(const std::string& tool, std::string& error);
  /**
   * @brief Solve the IK of all the goal poses and of the deviation poses with one task-redundant IK call.
   * @param goal Action goal.
   * @param seed Seed configuration.
   * @param[out] process IK solutions of each pose.
   * @param[out] deviation IK solutions of each pose moved by the deviation distance.
   * @param[out] error Error message.
   * @return False if the service call fails.
   */
  bool solveIk(const Action::Goal& goal, const Eigen::VectorXd& seed, std::vector<Candidates>& process,
               std::vector<Candidates>& deviation, std::string& error);

  using MultigoalAction = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan;
  using MultigoalClient = rclcpp_action::Client<MultigoalAction>;
  using TimeParametrizationAction = time_parametrization_msgs::action::ApplyTimeParametrization;

  /// Client of the planner selected by the multigoal_planner parameter (read at every goal).
  /// Returns nullptr and sets error if the parameter value is unknown.
  MultigoalClient::SharedPtr selectMultigoalClient(std::string& planner_name, std::string& error) const;

  /// Free motion from q_from to one of goals, planned by the multigoal action of client.
  /// path excludes q_from and ends exactly at goals[goal_index]. Cancels the request if gh is canceling.
  bool planMultigoal(const MultigoalClient::SharedPtr& client, const std::string& planner_name,
                     const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals,
                     std::vector<Eigen::VectorXd>& path, size_t& goal_index, const std::shared_ptr<GoalHandle>& gh);

  /// Asks the planner collision checker (/collision_check, detailed) why q_from and goals are rejected,
  /// and logs the colliding links. Used only to diagnose a failed planner request.
  void logCollisionDetails(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals);

  /**
   * @brief Split the path (preceded by q_start) at the first and at the last pose of every continuous
   * run of surface poses. SURFACE and FREE_MOTION paths alternate and share their end points.
   * Repeated consecutive configurations are removed; paths with less than two points are dropped.
   */
  void splitPath(const Eigen::VectorXd& q_start, const JointPathBuilderResult& path,
                 std::vector<trajectory_msgs::msg::JointTrajectory>& paths, std::vector<uint8_t>& path_types) const;

  /**
   * @brief Retime each path with its own time parametrization request (all sent at once) and append them.
   * @param goal compute_joint_trajectory goal (already validated).
   * @param[out] timed Timed trajectory, ordered as joint_names_.
   * @param gh Goal handle, for feedback and cancellation.
   * @param[out] error Error message.
   */
  bool timeParametrize(const TrajectoryAction::Goal& goal, trajectory_msgs::msg::JointTrajectory& timed,
                       const std::shared_ptr<TrajectoryGoalHandle>& gh, std::string& error);

  /**
   * @brief Publish a trajectory starting from q_start as a moveit_msgs/DisplayTrajectory for RViz.
   * @param timed If false, q_start is prepended and a fake time step (display_time_step) is used.
   */
  void publishDisplayTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                const Eigen::VectorXd& q_start, bool timed);

  /**
   * @brief Store the last joint state.
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Server<TrajectoryAction>::SharedPtr trajectory_server_;
  // Required by the graph_core collision checker API: it is only passed to
  // graph_core. All the messages of this package use the ROS 2 logger.
  graph::core::LoggerPtr graph_logger_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;

  // Separate callback groups: the goal runs on its own thread and waits for these responses.
  rclcpp::CallbackGroup::SharedPtr cb_group_services_;
  rclcpp::CallbackGroup::SharedPtr cb_group_joint_states_;
  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  rclcpp::Client<ik_solver_msgs::srv::ChangeTool>::SharedPtr change_tool_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  MultigoalClient::SharedPtr parallel_multigoal_client_;
  MultigoalClient::SharedPtr prm_multigoal_client_;
  rclcpp_action::Client<TimeParametrizationAction>::SharedPtr time_parametrization_client_;
  rclcpp::Client<ik_solver_msgs::srv::CollisionChecking>::SharedPtr collision_check_client_;

  std::mutex joint_state_mtx_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

  std::atomic<bool> busy_{ false };
  std::atomic<bool> trajectory_busy_{ false };

  // Robot model and collision checking
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  // Parameters
  std::string group_name_;
  double service_timeout_;
  double ik_timeout_;
  bool ik_parallel_;
  int ik_threads_;
  double joint_state_timeout_;
  double checker_resolution_;
  int checker_threads_;
  double display_time_step_;
  double multigoal_planning_time_;
  double time_parametrization_timeout_;
  // multigoal_planner is read at every goal.
  JointPathBuilderOptions builder_options_;
};

}  // namespace surface_path_planning
