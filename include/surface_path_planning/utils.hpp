#pragma once

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <Eigen/Core>

// ROS 2 and Logging
#include <rclcpp/rclcpp.hpp>

// Graph Core Base Plugin Interfaces
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/metrics/metrics_base_plugin.h>
#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>

#include <graph_core/logger.h>
#include <graph_core/params.h>

// Class Loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

namespace surface_path_planning {
namespace utils {

/**
 * @brief Reorders joint arrays (names, position, velocity, effort) in-place
 * to match a target ordering specified by order_names.
 * @return True if permutation succeeded, false otherwise (with optional diagnostic report).
 */
bool permutationName(const std::vector<std::string>& order_names,
                     std::vector<std::string>& names,
                     std::vector<double>& position,
                     std::vector<double>& velocity,
                     std::vector<double>& effort,
                     std::stringstream* report = nullptr);

/**
 * @brief Factory class responsible for dynamically loading and configuring
 * graph_core plugins using cnr_class_loader.
 */
class PlannerComponentsFactory {
public:
  /**
   * @brief Construct a new Planner Components Factory.
   * @param loader Reference to the multi-library class loader (must outlive the plugins).
   * @param node Shared pointer to the calling ROS 2 node for logging and parameter access.
   * @param logger Logger passed to the plugins.
   */
  PlannerComponentsFactory(cnr_class_loader::MultiLibraryClassLoader& loader,
                           const rclcpp::Node::SharedPtr& node,
                           const graph::core::LoggerPtr& logger);

  /**
   * @brief Dynamically instantiates and initializes the collision checker plugin.
   */
  graph::core::CollisionCheckerPtr createCollisionChecker(
      const graph::core::ParamMap& params,
      const planning_scene::PlanningScenePtr& scene);

  /**
   * @brief Dynamically instantiates and initializes the sampler plugin.
   */
  graph::core::SamplerPtr createSampler(
      const graph::core::ParamMap& params,
      const Eigen::VectorXd& start_conf,
      const Eigen::VectorXd& goal_conf,
      const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub);

  /**
   * @brief Dynamically instantiates and initializes the metrics (distance function) plugin.
   */
  graph::core::MetricsPtr createMetrics(const graph::core::ParamMap& params);

private:
  cnr_class_loader::MultiLibraryClassLoader& loader_;
  rclcpp::Node::SharedPtr node_;
  graph::core::LoggerPtr logger_;
};

} // namespace utils
} // namespace surface_path_planning