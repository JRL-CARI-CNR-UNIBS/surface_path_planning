// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa

#include "surface_path_planning/joint_path_builder.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <limits>


namespace surface_path_planning
{

JointPathBuilder::JointPathBuilder(const graph::core::CollisionCheckerPtr& checker, const Eigen::VectorXd& lower_bound,
                                   const Eigen::VectorXd& upper_bound, const rclcpp::Logger& logger,
                                   const MultigoalPlanner& planner, const JointPathBuilderOptions& options)
  : checker_(checker)
  , lower_bound_(lower_bound)
  , upper_bound_(upper_bound)
  , logger_(logger)
  , options_(options)
  , multigoal_planner_(planner)
{
}

bool JointPathBuilder::isValid(const Eigen::VectorXd& q) const
{
  if (q.size() != lower_bound_.size())
    return false;
  if ((q.array() < lower_bound_.array()).any() || (q.array() > upper_bound_.array()).any())
    return false;
  return checker_->check(q);
}

std::vector<Eigen::VectorXd> JointPathBuilder::sortedCandidates(const Eigen::VectorXd& q,
                                                                const Candidates& candidates) const
{
  // Pair (distance, candidate). Collision checks are postponed to the caller, which stops at the
  // first valid candidate: this avoids checking all the (possibly many) task-redundant solutions.
  std::vector<std::pair<double, const Eigen::VectorXd*>> scored;
  scored.reserve(candidates.size());
  for (const Eigen::VectorXd& c : candidates)
  {
    if (c.size() != q.size())
      continue;
    if ((c.array() < lower_bound_.array()).any() || (c.array() > upper_bound_.array()).any())
      continue;
    scored.emplace_back((c - q).norm(), &c);
  }
  std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<Eigen::VectorXd> sorted;
  sorted.reserve(scored.size());
  for (const auto& s : scored)
    sorted.push_back(*s.second);
  return sorted;
}

bool JointPathBuilder::pickDirect(const Eigen::VectorXd& q, const Candidates& candidates, double max_step,
                                  Eigen::VectorXd& q_next) const
{
  for (const Eigen::VectorXd& c : sortedCandidates(q, candidates))
  {
    // Sorted by Euclidean distance: candidates are not sorted by infinity norm, so keep looking.
    if ((c - q).cwiseAbs().maxCoeff() > max_step)
      continue;
    if (!checker_->check(c))
      continue;
    if (!checker_->checkConnection(q, c))
      continue;
    q_next = c;
    return true;
  }
  return false;
}

bool JointPathBuilder::planFreeMotionToAny(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals,
                                           std::vector<Eigen::VectorXd>& path, size_t& goal_index) const
{
  path.clear();
  if (goals.empty() || !multigoal_planner_)
    return false;
  if (!multigoal_planner_(q_from, goals, path, goal_index) || goal_index >= goals.size() || path.empty())
  {
    path.clear();
    return false;
  }
  return true;
}

bool JointPathBuilder::approach(const Eigen::VectorXd& q, size_t i, const std::vector<Candidates>& process,
                                const std::vector<Candidates>& deviation, std::vector<Eigen::VectorXd>& path) const
{
  const unsigned int max_goals = options_.max_multigoal_goals;

  // Approach configurations (closest first) with the surface configuration reachable from each one.
  std::vector<Eigen::VectorXd> approach_confs;
  std::vector<Eigen::VectorXd> point_confs;
  for (const Eigen::VectorXd& q_app : sortedCandidates(q, deviation.at(i)))
  {
    if (approach_confs.size() >= max_goals)
      break;
    if (!checker_->check(q_app))
      continue;

    // The approach configuration is useful only if the surface point is reachable from it.
    Eigen::VectorXd q_point;
    if (!pickDirect(q_app, process.at(i), options_.max_deviation_joint_step, q_point))
      continue;
    approach_confs.push_back(q_app);
    point_confs.push_back(q_point);
  }

  size_t k = 0;
  if (!planFreeMotionToAny(q, approach_confs, path, k))
  {
    RCLCPP_DEBUG_STREAM(logger_, "free motion to the approach configurations of pose " << i << " failed");
    return false;
  }
  path.push_back(point_confs.at(k));
  return true;
}

bool JointPathBuilder::leave(const Eigen::VectorXd& q, size_t i, const std::vector<Candidates>& deviation,
                             std::vector<Eigen::VectorXd>& path) const
{
  path.clear();
  Eigen::VectorXd q_dev;
  if (pickDirect(q, deviation.at(i), options_.max_deviation_joint_step, q_dev))
  {
    path.push_back(q_dev);
    return true;
  }

  // Fallback: plan a free motion to the closest deviation configurations.
  const unsigned int max_goals = options_.max_multigoal_goals;
  std::vector<Eigen::VectorXd> goals;
  for (const Eigen::VectorXd& c : sortedCandidates(q, deviation.at(i)))
  {
    if (goals.size() >= max_goals)
      break;
    if (checker_->check(c))
      goals.push_back(c);
  }
  size_t k = 0;
  return planFreeMotionToAny(q, goals, path, k);
}

bool JointPathBuilder::leaveWithBacktracking(JointPathBuilderResult& res, Eigen::VectorXd& q, size_t& last_feasible,
                                             const std::vector<Candidates>& deviation) const
{
  std::vector<unsigned int> dropped;
  std::vector<Eigen::VectorXd> path;
  while (true)
  {
    if (leave(q, last_feasible, deviation, path))
    {
      for (size_t k = 0; k < path.size(); k++)
      {
        res.waypoints.push_back(path.at(k));
        res.pose_indices.push_back(-1);
      }
      q = path.back();
      // Dropped poses are reported as skipped, keeping skipped_indices sorted.
      res.skipped_indices.insert(res.skipped_indices.end(), dropped.rbegin(), dropped.rend());
      return true;
    }

    // Backtrack only along the current segment: the previous waypoint must be a surface pose
    // (the waypoint before the first pose of a segment is its approach configuration).
    const size_t m = res.waypoints.size();
    if (m < 2 || res.pose_indices.at(m - 1) < 0 || res.pose_indices.at(m - 2) < 0 ||
        dropped.size() >= options_.max_backtracking_poses)
    {
      return false;
    }
    RCLCPP_INFO_STREAM(logger_, "unable to leave the surface from pose " << last_feasible << ": dropping it");
    dropped.push_back(static_cast<unsigned int>(res.pose_indices.back()));
    res.waypoints.pop_back();
    res.pose_indices.pop_back();
    q = res.waypoints.back();
    last_feasible = static_cast<size_t>(res.pose_indices.back());
  }
}

JointPathBuilderResult JointPathBuilder::build(const Eigen::VectorXd& q_start, const std::vector<Candidates>& process,
                                               const std::vector<Candidates>& deviation,
                                               const std::vector<bool>& segment_start,
                                               const ProgressCallback& progress)
{
  JointPathBuilderResult res;
  const size_t n = process.size();
  if (deviation.size() != n || segment_start.size() != n)
  {
    res.message = "process, deviation and segment_start must have the same size";
    return res;
  }
  if (q_start.size() != lower_bound_.size())
  {
    res.message = "start configuration has wrong size";
    return res;
  }
  if (!checker_->check(q_start))
  {
    res.message = "start configuration is in collision";
    return res;
  }

  auto append = [&res](const std::vector<Eigen::VectorXd>& path, int last_index) {
    for (size_t k = 0; k < path.size(); k++)
    {
      res.waypoints.push_back(path.at(k));
      res.pose_indices.push_back(k + 1 == path.size() ? last_index : -1);
    }
  };

  Eigen::VectorXd q = q_start;
  bool on_surface = false;  // true if q is the configuration of pose last_feasible
  size_t last_feasible = 0;
  std::vector<Eigen::VectorXd> path;

  for (size_t i = 0; i < n; i++)
  {
    if (progress && !progress(i, on_surface ? "moving along the surface" : "approaching the surface"))
    {
      res.message = "canceled";
      return res;
    }

    // 1) Continue on the surface, if possible.
    if (on_surface && !segment_start.at(i))
    {
      Eigen::VectorXd q_next;
      if (pickDirect(q, process.at(i), options_.max_joint_step, q_next))
      {
        append({ q_next }, static_cast<int>(i));
        q = q_next;
        last_feasible = i;
        continue;
      }
      RCLCPP_DEBUG_STREAM(logger_, "pose " << i << " is not reachable from pose " << last_feasible << ": implicit jump");
    }

    // 2) Explicit or implicit jump: leave the surface from the last feasible pose.
    if (on_surface)
    {
      if (!leaveWithBacktracking(res, q, last_feasible, deviation))
      {
        res.message = "unable to leave the surface from pose " + std::to_string(last_feasible);
        return res;
      }
      on_surface = false;
    }

    // 3) From free space, approach pose i.
    if (approach(q, i, process, deviation, path))
    {
      append(path, static_cast<int>(i));
      q = path.back();
      on_surface = true;
      last_feasible = i;
    }
    else
    {
      RCLCPP_INFO_STREAM(logger_, "pose " << i << " skipped: no feasible approach");
      res.skipped_indices.push_back(static_cast<unsigned int>(i));
    }
  }

  if (res.skipped_indices.size() == n)
  {
    res.message = "no pose is reachable";
    return res;
  }

  // Leave the surface at the end of the path.
  if (on_surface && !leaveWithBacktracking(res, q, last_feasible, deviation))
    RCLCPP_WARN_STREAM(logger_, "unable to leave the surface after the last pose " << last_feasible);

  res.success = true;
  res.message = "path computed, " + std::to_string(res.skipped_indices.size()) + " pose(s) skipped";
  return res;
}

}  // namespace surface_path_planning
