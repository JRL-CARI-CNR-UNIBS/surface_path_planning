// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/logger.hpp>

#include <graph_core/collision_checkers/collision_checker_base.h>

namespace surface_path_planning
{

/// All the IK solutions of one pose (each one ordered as the planning group joints).
using Candidates = std::vector<Eigen::VectorXd>;

/**
 * @brief Options of JointPathBuilder.
 */
struct JointPathBuilderOptions
{
  /// [rad] Max joint displacement (infinity norm) between two consecutive points of the same segment.
  double max_joint_step = 0.35;
  /// [rad] Max joint displacement (infinity norm) between a surface point and its leave/approach point.
  double max_deviation_joint_step = 0.8;
  /// Max number of surface poses dropped (going backwards along the segment) when the surface cannot be left.
  unsigned int max_backtracking_poses = 20;
  /// Max number of candidate goal configurations sent in one multigoal planning request.
  unsigned int max_multigoal_goals = 20;
};

/**
 * @brief Output of JointPathBuilder.
 */
struct JointPathBuilderResult
{
  bool success = false;  ///< True if a path was built (some poses may be skipped).
  std::string message;  ///< Description of the result or of the error.
  /// Joint configurations, in order, starting from the robot current configuration (excluded).
  std::vector<Eigen::VectorXd> waypoints;
  /// Same length as waypoints: index of the reached pose, -1 for auxiliary points.
  std::vector<int> pose_indices;
  /// Poses skipped because no feasible configuration was found.
  std::vector<unsigned int> skipped_indices;
};

/**
 * @brief Builds a joint-space path through a sequence of surface poses.
 *
 * Inputs are, for every pose i, the IK solutions of the pose itself (process[i]) and of the
 * pose moved by the deviation offset (deviation[i]). The path is built greedily:
 *
 *  - From free space (robot start or after leaving the surface), the robot approaches pose i:
 *    the closest approach configurations are all sent to the multigoal planner in one request,
 *    and it chooses which one to reach. Then the robot moves
 *    point-to-point from the approach configuration to the closest compatible IK of pose i.
 *  - Inside a segment, the robot moves to the closest IK of the next pose that is within
 *    max_joint_step and whose straight joint-space connection is collision free.
 *  - At an explicit jump, or when the next pose cannot be reached inside the segment (implicit
 *    jump), the robot leaves the surface moving to the deviation configuration of the last
 *    feasible pose, then it approaches the next pose as above. If the deviation configuration of that
 *    pose is not reachable, the last poses of the segment are dropped (and reported as skipped) until
 *    the robot can leave the surface.
 *  - A pose that cannot be approached is skipped. The path ends leaving the surface.
 *
 * Only point-to-point joint motions are produced: leave and approach motions are not Cartesian lines.
 */
class JointPathBuilder
{
public:
  /// Called before each pose: (pose index, status). Returning false cancels the computation.
  using ProgressCallback = std::function<bool(size_t, const std::string&)>;

  /**
   * Plans a free motion from q_from to any one of goals.
   * On success, path excludes q_from and ends exactly at goals[goal_index].
   */
  using MultigoalPlanner = std::function<bool(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals,
                                              std::vector<Eigen::VectorXd>& path, size_t& goal_index)>;

  /**
   * @param logger ROS 2 logger used for all the messages of this class
   * @param planner planner of the free motions (approach and leave)
   */
  JointPathBuilder(const graph::core::CollisionCheckerPtr& checker, const Eigen::VectorXd& lower_bound,
                   const Eigen::VectorXd& upper_bound, const rclcpp::Logger& logger,
                   const MultigoalPlanner& planner, const JointPathBuilderOptions& options);

  /**
   * @param q_start robot current configuration
   * @param process IK solutions of each pose
   * @param deviation IK solutions of each pose moved by the deviation offset
   * @param segment_start segment_start[i] is true if poses[i] starts a new segment (explicit jump)
   * @param progress optional progress/cancel callback
   */
  JointPathBuilderResult build(const Eigen::VectorXd& q_start, const std::vector<Candidates>& process,
                               const std::vector<Candidates>& deviation, const std::vector<bool>& segment_start,
                               const ProgressCallback& progress = nullptr);

private:
  bool isValid(const Eigen::VectorXd& q) const;

  /// Candidates within bounds, sorted by Euclidean joint distance from q.
  std::vector<Eigen::VectorXd> sortedCandidates(const Eigen::VectorXd& q, const Candidates& candidates) const;

  /// Closest candidate reachable from q with a collision-free straight motion of at most max_step.
  bool pickDirect(const Eigen::VectorXd& q, const Candidates& candidates, double max_step,
                  Eigen::VectorXd& q_next) const;

  /// Free motion from q_from to one of goals. path excludes q_from and ends at goals[goal_index].
  bool planFreeMotionToAny(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals,
                           std::vector<Eigen::VectorXd>& path, size_t& goal_index) const;

  /// From free space: approach configuration of pose i and then pose i.
  bool approach(const Eigen::VectorXd& q, size_t i, const std::vector<Candidates>& process,
                const std::vector<Candidates>& deviation, std::vector<Eigen::VectorXd>& path) const;

  /// Leaves the surface from the last pose of res. If it is not possible, drops the last poses of the
  /// segment (reported as skipped) and leaves from the previous one. Updates q and last_feasible.
  bool leaveWithBacktracking(JointPathBuilderResult& res, Eigen::VectorXd& q, size_t& last_feasible,
                             const std::vector<Candidates>& deviation) const;

  /// From pose i (configuration q): move to its deviation configuration.
  bool leave(const Eigen::VectorXd& q, size_t i, const std::vector<Candidates>& deviation,
             std::vector<Eigen::VectorXd>& path) const;

  graph::core::CollisionCheckerPtr checker_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  rclcpp::Logger logger_;
  JointPathBuilderOptions options_;
  MultigoalPlanner multigoal_planner_;
};

}  // namespace surface_path_planning
