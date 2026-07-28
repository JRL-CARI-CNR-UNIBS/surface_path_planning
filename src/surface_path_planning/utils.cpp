#include "surface_path_planning/utils.hpp"

namespace surface_path_planning {
namespace utils {

bool permutationName(const std::vector<std::string>& order_names,
                     std::vector<std::string>& names,
                     std::vector<double>& position,
                     std::vector<double>& velocity,
                     std::vector<double>& effort,
                     std::stringstream* report)
{
  if (names.size() < order_names.size()) {
    if (report) *report << "Vector size mismatch. Names to sort: " << names.size() << " < target: " << order_names.size();
    return false;
  }
  if (names.size() != position.size() || names.size() != velocity.size() || names.size() != effort.size()) {
    if (report) *report << "Input Mismatch. Parallel arrays do not share the same size.";
    return false;
  }

  for (unsigned int iOrder = 0; iOrder < order_names.size(); iOrder++) {
    if (names.at(iOrder).compare(order_names.at(iOrder))) {
      for (unsigned int iNames = iOrder + 1; iNames < names.size(); iNames++) {
        if (!order_names.at(iOrder).compare(names.at(iNames))) {
          std::iter_swap(names.begin() + iOrder,    names.begin() + iNames);
          std::iter_swap(position.begin() + iOrder, position.begin() + iNames);
          std::iter_swap(velocity.begin() + iOrder, velocity.begin() + iNames);
          std::iter_swap(effort.begin() + iOrder,   effort.begin() + iNames);
          break;
        }
        if (iNames == (names.size() - 1)) {
          if (report) *report << "The Joint '" << order_names.at(iOrder) << "' is missing in the input vector.";
          return false;
        }
      }
    }
  }
  return true;
}

PlannerComponentsFactory::PlannerComponentsFactory(cnr_class_loader::MultiLibraryClassLoader& loader,
                                                   const rclcpp::Node::SharedPtr& node,
                                                   const cnr_logger::TraceLoggerPtr& logger)
  : loader_(loader), node_(node), logger_(logger) {}

graph::core::CollisionCheckerPtr PlannerComponentsFactory::createCollisionChecker(
    const std::string& param_ns, const planning_scene::PlanningScenePtr& scene)
{
  std::string plugin_name;
  graph::core::get_param(logger_, param_ns, "checker_plugin", plugin_name,
                        (std::string)"graph::ros1::ParallelMoveitCollisionCheckerPlugin");

  RCLCPP_INFO(node_->get_logger(), "Loading collision checker plugin: %s", plugin_name.c_str());
  auto plugin = loader_.createInstance<graph::collision_check::MoveitCollisionCheckerBasePlugin>(plugin_name);
  plugin->init(param_ns, scene, logger_);
  return plugin->getCollisionChecker();
}

graph::core::SamplerPtr PlannerComponentsFactory::createSampler(
    const std::string& param_ns, const Eigen::VectorXd& start_conf,
    const Eigen::VectorXd& goal_conf, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{
  std::string plugin_name;
  graph::core::get_param(logger_, param_ns, "sampler_plugin", plugin_name,
                        (std::string)"graph::core::InformedSamplerPlugin");

  RCLCPP_INFO(node_->get_logger(), "Loading sampler plugin: %s", plugin_name.c_str());
  auto plugin = loader_.createInstance<graph::core::SamplerBasePlugin>(plugin_name);

  Eigen::VectorXd scale(start_conf.size());
  scale.setOnes(start_conf.size(), 1);

  plugin->init(param_ns, start_conf, goal_conf, lb, ub, scale, logger_);
  return plugin->getSampler();
}

graph::core::MetricsPtr PlannerComponentsFactory::createMetrics(const std::string& param_ns)
{
  std::string plugin_name;
  graph::core::get_param(logger_, param_ns, "metrics_plugin", plugin_name,
                        (std::string)"graph::core::EuclideanMetricsPlugin");

  RCLCPP_INFO(node_->get_logger(), "Loading metrics plugin: %s", plugin_name.c_str());
  auto plugin = loader_.createInstance<graph::core::MetricsBasePlugin>(plugin_name);
  plugin->init(param_ns, logger_);
  return plugin->getMetrics();
}

} // namespace utils
} // namespace surface_path_planning