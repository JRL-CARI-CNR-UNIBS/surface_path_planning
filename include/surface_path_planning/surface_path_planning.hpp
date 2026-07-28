#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

// MoveIt & Graph Core Includes
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <graph_core/solvers/rrt_star.h>
#include <graph_display/graph_display.h>
#include <ik_solver/internal/ik_solver_node.ros2.hpp>

// Utility factory
#include "surface_path_planning/utils.hpp"

namespace surface_path_planning {

class SurfacePlannerNode : public rclcpp::Node
{
public:
  SurfacePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Orchestrates setup, initializes plugins, maps environment parameters,
   * and executes the core RRT planning pipeline loop.
   */
  bool executePlanning(const std::string& robot_desc_string);

private:
  void loadParameters();
  bool initPlanningScene();
  bool initPlugins(const std::string& robot_desc_string);
  bool executeRrtLoop(graph::core::PathPtr& solution);

  // Core Configuration State
  int test_mode_ = 0;
  bool use_sharework_ = false;
  std::string package_name_ = "surface_path_planning";
  std::string param_ns2_;
  std::string group_name_;
  std::string end_effector_link_name_;

  // MoveIt Modeling & Scene tracking
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Framework Core Components & Solvers
  cnr_logger::TraceLoggerPtr logger_;
  cnr_class_loader::MultiLibraryClassLoader loader_;

  graph::core::CollisionCheckerPtr checker_;
  graph::core::SamplerPtr sampler_;
  graph::core::MetricsPtr metrics_;
  std::shared_ptr<ik_solver::IkSolver> ik_solver_;
  graph::display::DisplayPtr display_;
  graph::core::TreePtr tree_;

  // Joint Bounds and Config Targets
  Eigen::VectorXd lb_, ub_;
  Eigen::VectorXd start_conf_, goal_conf_;

  // Missing placeholder variables declared here as members to avoid compile failure.
  // These should be mapped to actual configuration constants or runtime params.
  Eigen::Affine3d T_w_b_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_f_ee_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_w_start_ = Eigen::Affine3d::Identity();
  int geometric_constraints_ = 0;

  // Random sampling engine variables
  std::mt19937 gen_{std::random_device{}()};
  std::uniform_real_distribution<double> dis_{0.0, 1.0};
};

} // namespace surface_path_planning