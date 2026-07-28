#include "surface_path_planning/surface_path_planning.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <fstream>

namespace surface_path_planning {

SurfacePlannerNode::SurfacePlannerNode(const rclcpp::NodeOptions& options)
  // Initialize the ROS2 node with a fixed runtime name ("test_solver") and
  // explicitly disable immediate plugin loading in `loader_` by passing `false`.
  // Plugins are loaded later in `initPlugins()` after parameters are available.
  : Node("test_solver", options), loader_(false) {}

bool SurfacePlannerNode::executePlanning(const std::string& robot_desc_string)
{
  // High-level pipeline:
  // 1) Read all runtime parameters and logger configuration.
  // 2) Build and synchronize the planning scene from move_group.
  // 3) Instantiate checker/sampler/metrics/IK plugins.
  // 4) Run the RRT loop until a valid path is found.
  loadParameters();
  if (!initPlanningScene()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize");
    return false;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Planning scene Ok");

  if (!initPlugins(robot_desc_string)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize plugins");
    return false;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "initPlugins Ok");


  // Read start/goal states from parameters and bootstrap tree endpoints.
  graph::core::get_param(logger_, param_ns2_, "start_configuration", start_conf_);
  graph::core::get_param(logger_, param_ns2_, "goal_configuration_1", goal_conf_);

  graph::core::PathPtr solution1;
  if (!executeRrtLoop(solution1)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute RRT loop");
    return false;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "RRT loop done.");
  display_->displayPathAndWaypoints(solution1, "path1_display",{1,0,0,1.0});
  display_->displayTree(tree_, "graph_display", {0.0, 0.0, 1.0, 0.15});


  //display_->displayPathAndWaypoints(solution2, "path2_display",{0,1,0,1.0});

  return true;
}

void SurfacePlannerNode::loadParameters()
{

  // `use_sharework` switches the end-effector link selection strategy.
  this->declare_parameter("use_sharework", false);
  use_sharework_ = this->get_parameter("use_sharework").as_bool();

  // Resolve package share path and bootstrap the logger from YAML config.
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name_);
  std::string logger_file = package_path + "/config/logger_param.yaml";
  logger_ = std::make_shared<cnr_logger::TraceLogger>("test_solver", logger_file);

  // Build parameter namespace used by graph_core helpers, then fetch the
  // MoveIt planning group name that identifies the active joint model group.
  param_ns2_ = "/" + package_name_ + "/test_solver";
  graph::core::get_param(logger_, param_ns2_, "group_name", group_name_);
  RCLCPP_INFO(this->get_logger(), "Planning group name: %s", group_name_.c_str());
}

bool SurfacePlannerNode::initPlanningScene()
{
  // Load robot model from `robot_description` and create a mutable planning scene.
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
  kinematic_model_ = robot_model_loader_->getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);

  // Extract active joint names for the selected planning group and compute
  // lower/upper joint bounds vectors used by the sampler and tree expansion.
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name_);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  // dof = degree of freedom
  unsigned int dof = joint_names.size();

  // lb = lower bound, ub = upper bound
  lb_.resize(dof); ub_.resize(dof);

  for (unsigned int idx = 0; idx < dof; idx++) {
    // joint_names[idx]   who cares...
    // joint_names.at(idx) raise an error if idx is bigger than the vector length

    //int a=5;
    //const int& b=a;
    //b=6;

    const auto& bounds = kinematic_model_->getVariableBounds(joint_names.at(idx));

    if (bounds.position_bounded_) {
      lb_(idx) = bounds.min_position_;
      ub_(idx) = bounds.max_position_;
    }
    else {
      lb_(idx) = -1000.0;
      ub_(idx) = 1000.0;
    }
  }



  // Retrieve remote scene configuration from the live move_group node
  auto ps_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!ps_client->wait_for_service(std::chrono::seconds(10))) return false;

  auto ps_srv = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto result = ps_client->async_send_request(ps_srv);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) != rclcpp::FutureReturnCode::SUCCESS) return false;

  // update planning_scene_ from the service response
  bool planning_scene_ok = planning_scene_->setPlanningSceneMsg(result.get()->scene);

  // Select the end-effector link:
  // - sharework flow: hardcoded "open_tip"
  // - default flow: last link from the robot model list
  end_effector_link_name_ = use_sharework_ ? "open_tip" : kinematic_model_->getLinkModelNames().back();
  // Build the visualization helper after planning scene and group are available.
  RCLCPP_INFO_STREAM(this->get_logger(), "End-effector link: " << end_effector_link_name_);
  display_ = std::make_shared<graph::display::Display>(shared_from_this(), planning_scene_, group_name_, end_effector_link_name_);

  return planning_scene_ok;
}

bool SurfacePlannerNode::initPlugins(const std::string& robot_desc_string)
{
  // Dynamically load all plugin libraries listed in parameters. This keeps the
  // planner extensible without hard-linking every implementation.
  std::vector<std::string> libraries;
  graph::core::get_param(logger_, param_ns2_, "libraries", libraries);
  for (const auto& lib : libraries) loader_.loadLibrary(lib);



  // Kinematics plugin architecture initialization:
  // load the UR10e IK plugin, connect a TF buffer/listener, inject the robot
  // description as parameter, then call plugin configuration entrypoint.
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");
  ik_solver_ = ik_loader.createSharedInstance("ik_solver/Ur10eIkSolver");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  ik_solver_->setBuffer(tf_buffer);

  std::string param_what;
  cnr::param::set("/ur_ik_solver/robot_description", robot_desc_string, param_what);
  if (!ik_solver_->config("/ur_ik_solver")) return false;




  return true;
}

bool SurfacePlannerNode::executeRrtLoop(graph::core::PathPtr& solution)
{



  // RRT tuning parameters:
  // - goal_bias: probability of sampling the exact goal configuration
  // - max_distance: expansion/connectivity threshold for tree extension
  double goal_bias;
  graph::core::get_param(logger_, param_ns2_, "goal_bias", goal_bias);
  double max_distance;
  graph::core::get_param(logger_, param_ns2_, "max_distance", max_distance);
  bool use_kdtree;
  graph::core::get_param(logger_, param_ns2_, "use_kdtree", use_kdtree);




  // Isolate implementation complexity using our Factory class
  utils::PlannerComponentsFactory factory(loader_, shared_from_this(), logger_);
  // collisioni
  checker_ = factory.createCollisionChecker(param_ns2_, planning_scene_);
  // genera punti random
  sampler_ = factory.createSampler(param_ns2_, start_conf_, goal_conf_, lb_, ub_);
  // misura la distanza
  metrics_ = factory.createMetrics(param_ns2_);

  // Validate that both boundary states are collision-free before starting RRT.
  // Failing early avoids growing a tree from invalid endpoints.
  if (!checker_->check(start_conf_) || !checker_->check(goal_conf_)) {
    RCLCPP_ERROR(this->get_logger(), "Boundary states checked inside obstacle spaces. Planning aborted.");
    return false;
  }

  auto goal_node = std::make_shared<graph::core::Node>(goal_conf_, logger_);
  auto start_node = std::make_shared<graph::core::Node>(start_conf_, logger_);



  // Tree grows from start, using configured metric/collision checker and
  // optional KD-tree acceleration (use_kdtree runtime parameter).
  tree_ = std::make_shared<graph::core::Tree>(start_node, max_distance, checker_, metrics_, logger_, use_kdtree);

  Eigen::VectorXd qrand;
  graph::core::NodePtr new_node;

  RCLCPP_INFO(this->get_logger(), "Starting RRT main evolution loop sequence...");


  bool solution_found = false;

  while (rclcpp::ok()) {
    // random value è un numero random fra 0 e 1
    double random_value = dis_(gen_);
    // Biased sampling:
    // with probability `goal_bias` pick the goal directly; otherwise sample
    // from the configured sampler strategy.
    if (random_value < goal_bias) {
      qrand = goal_conf_;
    } else {
      qrand = sampler_->sample();
    }

    // Try to extend the tree toward the sampled target configuration.
    if (tree_->extend(qrand, new_node)) {
      auto new_conf = new_node->getConfiguration();
      std::stringstream report; double value;

      // If the new node is close enough to goal, verify straight-line
      // connection. On success, connect goal, reconstruct path, and publish
      // both path and exploration tree for visualization.
      if ((new_node->getConfiguration() - goal_conf_).norm() < max_distance) {
        if (checker_->checkConnection(new_node->getConfiguration(), goal_conf_)) {
          tree_->extend(qrand, goal_node);
          RCLCPP_INFO(this->get_logger(), "Goal node successfully connected to the exploration tree!");

          solution = std::make_shared<graph::core::Path>(tree_->getConnectionToNode(goal_node), metrics_, checker_, logger_);
          solution->setTree(tree_);
          RCLCPP_INFO_STREAM(this->get_logger(), "Solution successfully created!" << *solution);

          solution_found = true;
          break;
        }
      }
    }
  }
  return solution_found;
}


} // namespace surface_path_planning
