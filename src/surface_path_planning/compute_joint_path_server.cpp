// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa

#include "surface_path_planning/compute_joint_path_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <thread>
#include <unordered_map>

#include <Eigen/Geometry>

#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <rclcpp/version.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace surface_path_planning
{

namespace
{
// Humble's create_client() only takes rmw_qos_profile_t; Jazzy (rclcpp 28+) takes rclcpp::QoS
// and deprecates the rmw overload.
#if RCLCPP_VERSION_MAJOR >= 28
rclcpp::QoS serviceQoS() {return rclcpp::ServicesQoS();}
#else
const rmw_qos_profile_t & serviceQoS() {return rmw_qos_profile_services_default;}
#endif

/// Reorders (names, values) following order. Returns false if a joint is missing.
bool reorder(const std::vector<std::string>& names, const std::vector<double>& values,
             const std::vector<std::string>& order, Eigen::VectorXd& out)
{
  if (names.size() != values.size())
    return false;
  std::unordered_map<std::string, double> map;
  for (size_t i = 0; i < names.size(); i++)
    map[names[i]] = values[i];
  out.resize(static_cast<Eigen::Index>(order.size()));
  for (size_t i = 0; i < order.size(); i++)
  {
    auto it = map.find(order[i]);
    if (it == map.end())
      return false;
    out(static_cast<Eigen::Index>(i)) = it->second;
  }
  return true;
}

/// Waits for a service response without spinning (the node is spun by the executor).
template <typename FutureT>
bool waitFor(FutureT& future, double timeout)
{
  return future.wait_for(std::chrono::duration<double>(timeout)) == std::future_status::ready;
}
}  // namespace

ComputeJointPathServer::ComputeJointPathServer(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  group_name_ = node_->declare_parameter<std::string>("group_name", "manipulator");
  const std::string ik_ns = node_->declare_parameter<std::string>("ik_solver_namespace", "/ur_ik_solver");
  const std::string joint_states_topic = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  const std::string scene_service =
      node_->declare_parameter<std::string>("planning_scene_service", "/get_planning_scene");

  service_timeout_ = node_->declare_parameter<double>("service_timeout", 10.0);
  ik_timeout_ = node_->declare_parameter<double>("ik_timeout", 120.0);
  // Parallel IK: the UR solver is analytic and ignores seeds, so nothing is lost by not chaining them.
  ik_parallel_ = node_->declare_parameter<bool>("ik_parallel", true);
  ik_threads_ = std::max<int>(0, node_->declare_parameter<int>("ik_threads", 0));
  joint_state_timeout_ = node_->declare_parameter<double>("joint_state_timeout", 5.0);
  checker_resolution_ = node_->declare_parameter<double>("checker_resolution", 0.02);
  checker_threads_ = node_->declare_parameter<int>("checker_threads", 4);

  builder_options_.max_joint_step = node_->declare_parameter<double>("max_joint_step", 0.35);
  builder_options_.max_deviation_joint_step = node_->declare_parameter<double>("max_deviation_joint_step", 0.8);
  builder_options_.max_backtracking_poses =
      static_cast<unsigned int>(node_->declare_parameter<int>("max_backtracking_poses", 20));
  builder_options_.max_multigoal_goals =
      static_cast<unsigned int>(node_->declare_parameter<int>("max_multigoal_goals", 20));

  // Free motions are planned by parallel_multigoal_planner or prm_multigoal_planner (same action type).
  // multigoal_planner is read at every goal, so it can be changed with `ros2 param set`.
  node_->declare_parameter<std::string>("multigoal_planner", "parallel_multigoal_planner");
  const std::string parallel_multigoal_action =
      node_->declare_parameter<std::string>("parallel_multigoal_action", "/parallel_multigoal_plan");
  const std::string prm_multigoal_action =
      node_->declare_parameter<std::string>("prm_multigoal_action", "/prm_multigoal_plan");
  multigoal_planning_time_ = node_->declare_parameter<double>("multigoal_planning_time", 5.0);

  // Time parametrization server (TOTG) used by the compute_joint_trajectory action.
  const std::string time_parametrization_action =
      node_->declare_parameter<std::string>("time_parametrization_action", "/apply_time_parametrization");
  time_parametrization_timeout_ = node_->declare_parameter<double>("time_parametrization_timeout", 30.0);
  // Collision checker used by parallel_multigoal_planner: queried only to explain a failed request.
  const std::string collision_check_service =
      node_->declare_parameter<std::string>("collision_check_service", "/collision_check");

  // Messages of graph_core go to the child logger "<node>.graph_core". Its default level is WARN;
  // it can be changed with e.g. --ros-args --log-level compute_joint_path.graph_core:=debug
  const rclcpp::Logger graph_logger = node_->get_logger().get_child("graph_core");
  if (rcutils_logging_get_logger_level(graph_logger.get_name()) == RCUTILS_LOG_SEVERITY_UNSET)
    rcutils_logging_set_logger_level(graph_logger.get_name(), RCUTILS_LOG_SEVERITY_WARN);
  graph_logger_ = std::make_shared<RclcppGraphLogger>(graph_logger);

  // Joint path for Rviz (MoveIt "Trajectory" display). Latched: the last path is always available.
  display_time_step_ = node_->declare_parameter<double>("display_time_step", 1.0);
  const std::string display_topic = node_->declare_parameter<std::string>("display_topic", "~/display_path");
  display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      display_topic, rclcpp::QoS(1).transient_local().reliable());

  std::string ns = ik_ns;
  if (!ns.empty() && ns.back() == '/')
    ns.pop_back();

  cb_group_services_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_joint_states_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ik_client_ = node_->create_client<ik_solver_msgs::srv::GetIkArray>(ns + "/get_task_redundant_ik_array",
                                                                     serviceQoS(), cb_group_services_);
  change_tool_client_ =
      node_->create_client<ik_solver_msgs::srv::ChangeTool>(ns + "/change_tool", serviceQoS(), cb_group_services_);
  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(scene_service, serviceQoS(),
                                                                           cb_group_services_);

  parallel_multigoal_client_ =
      rclcpp_action::create_client<MultigoalAction>(node_, parallel_multigoal_action, cb_group_services_);
  prm_multigoal_client_ =
      rclcpp_action::create_client<MultigoalAction>(node_, prm_multigoal_action, cb_group_services_);
  time_parametrization_client_ = rclcpp_action::create_client<TimeParametrizationAction>(
      node_, time_parametrization_action, cb_group_services_);
  collision_check_client_ = node_->create_client<ik_solver_msgs::srv::CollisionChecking>(
      collision_check_service, serviceQoS(), cb_group_services_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_joint_states_;
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      std::bind(&ComputeJointPathServer::jointStateCallback, this, std::placeholders::_1), sub_options);
}

bool ComputeJointPathServer::init()
{
  // Kinematics plugins are not needed: IK comes from the ik_solver node.
  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", false);
  robot_model_ = robot_model_loader_->getModel();
  if (!robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to load the robot model");
    return false;
  }
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning group '%s' not found", group_name_.c_str());
    return false;
  }
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  joint_names_ = jmg->getActiveJointModelNames();
  const size_t dof = joint_names_.size();
  lower_bound_.resize(dof);
  upper_bound_.resize(dof);
  for (size_t i = 0; i < dof; i++)
  {
    const auto& bounds = robot_model_->getVariableBounds(joint_names_.at(i));
    // Continuous joints are limited to one turn per side, to keep sampling meaningful.
    lower_bound_(i) = bounds.position_bounded_ ? bounds.min_position_ : -2.0 * M_PI;
    upper_bound_(i) = bounds.position_bounded_ ? bounds.max_position_ : 2.0 * M_PI;
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  server_ = rclcpp_action::create_server<Action>(node_, "compute_joint_path",
                                                 std::bind(&ComputeJointPathServer::handleGoal, this, _1, _2),
                                                 std::bind(&ComputeJointPathServer::handleCancel, this, _1),
                                                 std::bind(&ComputeJointPathServer::handleAccepted, this, _1));

  trajectory_server_ = rclcpp_action::create_server<TrajectoryAction>(
      node_, "compute_joint_trajectory", std::bind(&ComputeJointPathServer::handleTrajectoryGoal, this, _1, _2),
      [](const std::shared_ptr<TrajectoryGoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      std::bind(&ComputeJointPathServer::handleTrajectoryAccepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
              "compute_joint_path and compute_joint_trajectory action servers ready (group '%s', %zu joints)",
              group_name_.c_str(), dof);
  return true;
}

void ComputeJointPathServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mtx_);
  // Some drivers publish several partial joint state messages: keep only the complete ones.
  Eigen::VectorXd q;
  if (reorder(msg->name, msg->position, joint_names_, q))
    last_joint_state_ = msg;
}

rclcpp_action::GoalResponse ComputeJointPathServer::handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                               std::shared_ptr<const Action::Goal> goal)
{
  if (goal->poses.poses.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected: no poses");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->planning_frame.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected: planning_frame is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->poses.header.frame_id.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected: poses.header.frame_id is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (uint32_t idx : goal->jump_indices)
  {
    if (idx >= goal->poses.poses.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected: jump index %u out of range", idx);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  bool expected = false;
  // compare_exchange_strong returns false if busy_ is not equal to expected (i.e. if busy_ is true). In that case, the goal is rejected.
  // Thread-safe: It is an atomic operation.
  if (!busy_.compare_exchange_strong(expected, true))
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected: another goal is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ComputeJointPathServer::handleCancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ComputeJointPathServer::handleAccepted(const std::shared_ptr<GoalHandle> gh)
{
  std::thread{ [this, gh]() {
    execute(gh);
    busy_ = false;
  } }.detach();
}

bool ComputeJointPathServer::getCurrentConfiguration(Eigen::VectorXd& q, std::string& error)
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> lock(joint_state_mtx_);
      if (last_joint_state_ && reorder(last_joint_state_->name, last_joint_state_->position, joint_names_, q))
        return true;
    }
    if (std::chrono::steady_clock::now() - start > std::chrono::duration<double>(joint_state_timeout_))
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  error = "no complete joint state received";
  return false;
}

bool ComputeJointPathServer::updatePlanningScene(std::string& error)
{
  if (!scene_client_->wait_for_service(std::chrono::duration<double>(service_timeout_)))
  {
    error = "planning scene service not available";
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  using C = moveit_msgs::msg::PlanningSceneComponents;
  req->components.components = C::SCENE_SETTINGS | C::ROBOT_STATE | C::ROBOT_STATE_ATTACHED_OBJECTS |
                               C::WORLD_OBJECT_NAMES | C::WORLD_OBJECT_GEOMETRY | C::OCTOMAP | C::TRANSFORMS |
                               C::ALLOWED_COLLISION_MATRIX | C::LINK_PADDING_AND_SCALING | C::OBJECT_COLORS;
  auto future = scene_client_->async_send_request(req);
  if (!waitFor(future, service_timeout_))
  {
    error = "planning scene service timeout";
    return false;
  }
  if (!planning_scene_->setPlanningSceneMsg(future.get()->scene))
  {
    error = "unable to apply the planning scene";
    return false;
  }
  return true;
}

bool ComputeJointPathServer::changeTool(const std::string& tool, std::string& error)
{
  if (!change_tool_client_->wait_for_service(std::chrono::duration<double>(service_timeout_)))
  {
    error = "ik_solver change_tool service not available";
    return false;
  }
  auto req = std::make_shared<ik_solver_msgs::srv::ChangeTool::Request>();
  req->tool = tool;
  auto future = change_tool_client_->async_send_request(req);
  if (!waitFor(future, service_timeout_))
  {
    error = "ik_solver change_tool timeout";
    return false;
  }
  if (future.get()->result != ik_solver_msgs::srv::ChangeTool::Response::SUCCESS)
  {
    error = "ik_solver cannot use '" + tool + "' as tool frame (result " + std::to_string(future.get()->result) + ")";
    return false;
  }
  return true;
}

bool ComputeJointPathServer::solveIk(const Action::Goal& goal, const Eigen::VectorXd& seed,
                                     std::vector<Candidates>& process, std::vector<Candidates>& deviation,
                                     std::string& error)
{
  if (!ik_client_->wait_for_service(std::chrono::duration<double>(service_timeout_)))
  {
    error = "ik_solver get_task_redundant_ik_array service not available";
    return false;
  }

  const size_t n = goal.poses.poses.size();
  const Eigen::Vector3d offset(goal.deviation_distance.x, goal.deviation_distance.y, goal.deviation_distance.z);

  ik_solver_msgs::msg::Configuration seed_msg;
  seed_msg.configuration.assign(seed.data(), seed.data() + seed.size());

  // Targets 0..n-1: the poses. Targets n..2n-1: the poses moved by the offset in their own (tool) frame.
  auto req = std::make_shared<ik_solver_msgs::srv::GetIkArray::Request>();
  req->seed_joint_names = joint_names_;
  req->parallelize = ik_parallel_ ? ik_solver_msgs::srv::GetIkArray::Request::PARALLELIZE_FORCE
                                  : ik_solver_msgs::srv::GetIkArray::Request::PARALLELIZE_DISABLE;
  req->num_threads = static_cast<uint32_t>(ik_threads_);
  req->targets.resize(2 * n);
  for (size_t i = 0; i < n; i++)
  {
    Eigen::Isometry3d T_frame_tool;
    tf2::fromMsg(goal.poses.poses.at(i), T_frame_tool);
    Eigen::Isometry3d T_frame_dev = T_frame_tool * Eigen::Translation3d(offset);

    for (size_t k = 0; k < 2; k++)
    {
      auto& target = req->targets.at(i + k * n);
      target.pose.header = goal.poses.header;
      target.pose.pose = tf2::toMsg(k == 0 ? T_frame_tool : T_frame_dev);
      target.seeds.push_back(seed_msg);
    }
  }

  auto future = ik_client_->async_send_request(req);
  if (!waitFor(future, ik_timeout_))
  {
    error = "ik_solver timeout";
    return false;
  }
  auto res = future.get();
  if (res->solutions.size() != 2 * n)
  {
    error = "ik_solver returned " + std::to_string(res->solutions.size()) + " solutions instead of " +
            std::to_string(2 * n) + " (is the poses frame known to TF?)";
    return false;
  }

  process.assign(n, Candidates());
  deviation.assign(n, Candidates());
  for (size_t t = 0; t < 2 * n; t++)
  {
    Candidates& candidates = t < n ? process.at(t) : deviation.at(t - n);
    for (const auto& conf : res->solutions.at(t).configurations)
    {
      Eigen::VectorXd q;
      if (reorder(res->joint_names, conf.configuration, joint_names_, q))
        candidates.push_back(q);
    }
  }
  return true;
}

ComputeJointPathServer::MultigoalClient::SharedPtr
ComputeJointPathServer::selectMultigoalClient(std::string& planner_name, std::string& error) const
{
  planner_name = node_->get_parameter("multigoal_planner").as_string();
  if (planner_name == "parallel_multigoal_planner")
    return parallel_multigoal_client_;
  if (planner_name == "prm_multigoal_planner")
    return prm_multigoal_client_;
  error = "unknown multigoal_planner '" + planner_name +
          "' (allowed values: parallel_multigoal_planner, prm_multigoal_planner)";
  return nullptr;
}

bool ComputeJointPathServer::planMultigoal(const MultigoalClient::SharedPtr& client, const std::string& planner_name,
                                           const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals,
                                           std::vector<Eigen::VectorXd>& path, size_t& goal_index,
                                           const std::shared_ptr<GoalHandle>& gh)
{
  path.clear();
  const auto logger = node_->get_logger();
  const char* name = planner_name.c_str();
  if (!client->wait_for_action_server(std::chrono::duration<double>(service_timeout_)))
  {
    RCLCPP_ERROR(logger, "%s action server not available", name);
    return false;
  }

  // Every goal is an alternative joint-space goal (goal_constraints are OR'd).
  MultigoalAction::Goal goal;
  auto& req = goal.motion_plan_request;
  req.group_name = group_name_;
  req.allowed_planning_time = multigoal_planning_time_;
  req.start_state.joint_state.name = joint_names_;
  req.start_state.joint_state.position.assign(q_from.data(), q_from.data() + q_from.size());
  for (const Eigen::VectorXd& g : goals)
  {
    moveit_msgs::msg::Constraints c;
    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint_names_.at(j);
      jc.position = g(static_cast<Eigen::Index>(j));
      jc.tolerance_above = 1e-4;
      jc.tolerance_below = 1e-4;
      jc.weight = 1.0;
      c.joint_constraints.push_back(jc);
    }
    req.goal_constraints.push_back(c);
  }

  auto goal_handle_future = client->async_send_goal(goal);
  if (!waitFor(goal_handle_future, service_timeout_) || !goal_handle_future.get())
  {
    RCLCPP_ERROR(logger, "%s rejected the goal (or did not answer)", name);
    return false;
  }
  auto planner_gh = goal_handle_future.get();
  auto result_future = client->async_get_result(planner_gh);

  // Wait for the result, forwarding cancellation of our own goal.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(multigoal_planning_time_ + service_timeout_);
  while (!waitFor(result_future, 0.05))
  {
    if (gh->is_canceling() || std::chrono::steady_clock::now() > deadline || !rclcpp::ok())
    {
      if (!gh->is_canceling())
        RCLCPP_ERROR(logger, "%s timeout", name);
      client->async_cancel_goal(planner_gh);
      return false;
    }
  }

  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_WARN(logger, "%s failed (MoveIt error code %d)", name,
                wrapped.result ? wrapped.result->motion_plan_response.error_code.val : 0);
    logCollisionDetails(q_from, goals);
    return false;
  }

  const auto& traj = wrapped.result->motion_plan_response.trajectory.joint_trajectory;
  for (const auto& point : traj.points)
  {
    Eigen::VectorXd q;
    if (!reorder(traj.joint_names, point.positions, joint_names_, q))
    {
      RCLCPP_ERROR(logger, "%s returned a trajectory with different joints", name);
      path.clear();
      return false;
    }
    // The trajectory starts from q_from (possibly repeated): skip it.
    if (path.empty() && (q - q_from).norm() < 1e-6)
      continue;
    path.push_back(q);
  }
  if (path.empty())
    return false;

  // The last point is the reached goal: find which one.
  double best = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < goals.size(); k++)
  {
    const double d = (goals.at(k) - path.back()).norm();
    if (d < best)
    {
      best = d;
      goal_index = k;
    }
  }
  if (best > 1e-3)
  {
    RCLCPP_ERROR(logger, "%s path does not end at a requested goal (distance %f)", name, best);
    path.clear();
    return false;
  }
  path.back() = goals.at(goal_index);
  return true;
}

void ComputeJointPathServer::logCollisionDetails(const Eigen::VectorXd& q_from, const std::vector<Eigen::VectorXd>& goals)
{
  using ik_solver_msgs::msg::CollisionResult;
  const auto logger = node_->get_logger();
  // Do not wait: the checker may not be running (prm_multigoal_planner does not need it).
  if (!collision_check_client_->service_is_ready())
  {
    RCLCPP_WARN(logger, "collision check service '%s' not available: cannot explain the failure",
                collision_check_client_->get_service_name());
    return;
  }

  // Index 0: start configuration. Indices 1..n: goals.
  auto req = std::make_shared<ik_solver_msgs::srv::CollisionChecking::Request>();
  req->joint_names = joint_names_;
  req->detailed = true;
  auto add = [&req](const Eigen::VectorXd& q) {
    ik_solver_msgs::msg::Configuration c;
    c.configuration.assign(q.data(), q.data() + q.size());
    req->solutions.push_back(c);
  };
  add(q_from);
  for (const Eigen::VectorXd& g : goals)
    add(g);

  auto future = collision_check_client_->async_send_request(req);
  if (!waitFor(future, service_timeout_))
  {
    RCLCPP_WARN(logger, "collision check service timeout: cannot explain the failure");
    return;
  }
  const auto res = future.get();
  if (res->result.size() != req->solutions.size())
  {
    RCLCPP_WARN(logger, "collision check returned %zu results instead of %zu (joint names mismatch?)",
                res->result.size(), req->solutions.size());
    return;
  }

  auto describe = [](const CollisionResult& r) -> std::string {
    std::string s;
    if (r.out_of_bound)
    {
      s = "out of bounds:";
      for (const auto& j : r.out_of_bound_links)
        s += " " + j;
      return s;
    }
    if (r.feasibility > 0.0)
      return "collision free";
    for (size_t k = 0; k < r.colliding_obj_first.size() && k < r.colliding_obj_second.size(); k++)
      s += (s.empty() ? "" : ", ") + r.colliding_obj_first.at(k) + " <-> " + r.colliding_obj_second.at(k);
    return s.empty() ? "in collision (no contact reported)" : "contacts: " + s;
  };

  std::ostringstream q_str;
  q_str << q_from.transpose();
  RCLCPP_WARN(logger, "start configuration [%s]: %s", q_str.str().c_str(), describe(res->result.at(0)).c_str());

  // Goals: group identical explanations to keep the log short.
  std::map<std::string, size_t> reasons;
  size_t rejected = 0;
  for (size_t k = 1; k < res->result.size(); k++)
  {
    if (res->result.at(k).feasibility > 0.0)
      continue;
    rejected++;
    reasons[describe(res->result.at(k))]++;
  }
  RCLCPP_WARN(logger, "%zu/%zu goal configurations rejected by the collision check", rejected, goals.size());
  for (const auto& r : reasons)
    RCLCPP_WARN(logger, "  %zu goal(s): %s", r.second, r.first.c_str());
}

void ComputeJointPathServer::splitPath(const Eigen::VectorXd& q_start, const JointPathBuilderResult& path,
                                       std::vector<trajectory_msgs::msg::JointTrajectory>& paths,
                                       std::vector<uint8_t>& path_types) const
{
  paths.clear();
  path_types.clear();

  // Whole path, starting from the current configuration. surface[k]: point k is a surface pose.
  std::vector<Eigen::VectorXd> points{ q_start };
  std::vector<bool> surface{ false };
  points.insert(points.end(), path.waypoints.begin(), path.waypoints.end());
  for (int idx : path.pose_indices)
    surface.push_back(idx >= 0);
  const size_t n = points.size();

  // Break points: path ends, first and last pose of every run of surface poses.
  std::vector<size_t> breaks{ 0 };
  for (size_t k = 1; k < n; k++)
  {
    const bool run_start = surface.at(k) && !surface.at(k - 1);
    const bool run_end = surface.at(k) && (k + 1 == n || !surface.at(k + 1));
    if (run_start || run_end || k + 1 == n)
      breaks.push_back(k);
  }

  for (size_t b = 0; b + 1 < breaks.size(); b++)
  {
    trajectory_msgs::msg::JointTrajectory piece;
    piece.joint_names = joint_names_;
    bool on_surface = true;
    const Eigen::VectorXd* last = nullptr;
    for (size_t k = breaks.at(b); k <= breaks.at(b + 1); k++)
    {
      on_surface = on_surface && surface.at(k);
      // Repeated configurations make TOTG fail: drop them.
      if (last && (points.at(k) - *last).norm() <= 1e-9)
        continue;
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(points.at(k).data(), points.at(k).data() + points.at(k).size());
      piece.points.push_back(p);
      last = &points.at(k);
    }
    if (piece.points.size() < 2)
      continue;
    paths.push_back(piece);
    path_types.push_back(on_surface ? Action::Result::SURFACE : Action::Result::FREE_MOTION);
  }
}

rclcpp_action::GoalResponse
ComputeJointPathServer::handleTrajectoryGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                             std::shared_ptr<const TrajectoryAction::Goal> goal)
{
  if (goal->paths.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "compute_joint_trajectory goal rejected: no paths");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->path_types.size() != goal->paths.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "compute_joint_trajectory goal rejected: %zu path_types for %zu paths",
                 goal->path_types.size(), goal->paths.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!trajectory_busy_.compare_exchange_strong(expected, true))
  {
    RCLCPP_ERROR(node_->get_logger(), "compute_joint_trajectory goal rejected: another goal is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ComputeJointPathServer::handleTrajectoryAccepted(const std::shared_ptr<TrajectoryGoalHandle> gh)
{
  std::thread{ [this, gh]() {
    executeTrajectory(gh);
    trajectory_busy_ = false;
  } }.detach();
}

void ComputeJointPathServer::executeTrajectory(const std::shared_ptr<TrajectoryGoalHandle> gh)
{
  const auto goal = gh->get_goal();
  auto result = std::make_shared<TrajectoryAction::Result>();

  std::string error;
  trajectory_msgs::msg::JointTrajectory timed;
  if (!timeParametrize(*goal, timed, gh, error))
  {
    result->success = false;
    result->message = error;
    if (gh->is_canceling())
    {
      result->message = "canceled";
      gh->canceled(result);
      return;
    }
    RCLCPP_ERROR(node_->get_logger(), "compute_joint_trajectory failed: %s", error.c_str());
    gh->abort(result);
    return;
  }

  timed.header.frame_id = goal->paths.front().header.frame_id;
  result->trajectory.joint_trajectory = timed;
  result->success = true;
  result->message = "trajectory computed, " + std::to_string(timed.points.size()) + " points, " +
                    std::to_string(rclcpp::Duration(timed.points.back().time_from_start).seconds()) + " s";

  const auto& p0 = timed.points.front().positions;
  publishDisplayTrajectory(timed, Eigen::Map<const Eigen::VectorXd>(p0.data(), static_cast<Eigen::Index>(p0.size())),
                           true);
  gh->succeed(result);
}

bool ComputeJointPathServer::timeParametrize(const TrajectoryAction::Goal& goal,
                                             trajectory_msgs::msg::JointTrajectory& timed,
                                             const std::shared_ptr<TrajectoryGoalHandle>& gh, std::string& error)
{
  timed = trajectory_msgs::msg::JointTrajectory();
  timed.joint_names = joint_names_;

  auto feedback = std::make_shared<TrajectoryAction::Feedback>();
  feedback->total_paths = static_cast<uint32_t>(goal.paths.size());
  auto publishFeedback = [&](size_t index, const std::string& status) {
    feedback->current_index = static_cast<uint32_t>(index);
    feedback->status = status;
    gh->publish_feedback(feedback);
  };

  // Paths in joint_names_ order, without repeated points, checking that they are consecutive.
  struct Piece
  {
    std::vector<Eigen::VectorXd> points;
    bool surface;
    size_t index;  // index in goal.paths
  };
  std::vector<Piece> pieces;
  const Eigen::VectorXd* previous_end = nullptr;
  for (size_t i = 0; i < goal.paths.size(); i++)
  {
    const uint8_t type = goal.path_types.at(i);
    if (type != TrajectoryAction::Goal::SURFACE && type != TrajectoryAction::Goal::FREE_MOTION)
    {
      error = "path " + std::to_string(i) + ": unknown path type " + std::to_string(type);
      return false;
    }
    Piece piece{ {}, type == TrajectoryAction::Goal::SURFACE, i };
    for (const auto& point : goal.paths.at(i).points)
    {
      Eigen::VectorXd q;
      if (!reorder(goal.paths.at(i).joint_names, point.positions, joint_names_, q))
      {
        error = "path " + std::to_string(i) + ": joint names do not match the planning group '" + group_name_ + "'";
        return false;
      }
      if (piece.points.empty() || (q - piece.points.back()).norm() > 1e-9)
        piece.points.push_back(q);
    }
    if (piece.points.empty())
      continue;
    if (previous_end && (piece.points.front() - *previous_end).cwiseAbs().maxCoeff() > 1e-4)
    {
      error = "path " + std::to_string(i) + " does not start at the end of the previous path";
      return false;
    }
    pieces.push_back(piece);
    previous_end = &pieces.back().points.back();
  }
  pieces.erase(std::remove_if(pieces.begin(), pieces.end(), [](const Piece& p) { return p.points.size() < 2; }),
               pieces.end());
  if (pieces.empty())
  {
    error = "the paths contain no motion";
    return false;
  }

  publishFeedback(0, "waiting for the time parametrization server");
  if (!time_parametrization_client_->wait_for_action_server(std::chrono::duration<double>(service_timeout_)))
  {
    error = "time parametrization action server '" +
            node_->get_parameter("time_parametrization_action").as_string() + "' not available";
    return false;
  }

  // Send all the requests at once: the pieces are independent.
  using GoalHandleTp = rclcpp_action::ClientGoalHandle<TimeParametrizationAction>;
  std::vector<std::shared_future<GoalHandleTp::SharedPtr>> goal_futures;
  for (const Piece& piece : pieces)
  {
    TimeParametrizationAction::Goal tp_goal;
    auto& plan = tp_goal.unparametrized_motion_plan_response;
    plan.group_name = group_name_;
    plan.trajectory_start.joint_state.name = joint_names_;
    plan.trajectory_start.joint_state.position.assign(piece.points.front().data(),
                                                      piece.points.front().data() + piece.points.front().size());
    auto& traj = plan.trajectory.joint_trajectory;
    traj.joint_names = joint_names_;
    for (const Eigen::VectorXd& q : piece.points)
    {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(q.data(), q.data() + q.size());
      traj.points.push_back(p);
    }
    tp_goal.max_velocity_scaling_factor =
        piece.surface ? goal.surface_velocity_scaling : goal.free_motion_velocity_scaling;
    tp_goal.max_acceleration_scaling_factor =
        piece.surface ? goal.surface_acceleration_scaling : goal.free_motion_acceleration_scaling;
    goal_futures.push_back(time_parametrization_client_->async_send_goal(tp_goal));
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(time_parametrization_timeout_);
  // Waits for a future, stopping if the goal is canceled or the deadline expires.
  auto wait = [&](auto& future) {
    while (!waitFor(future, 0.05))
    {
      if (gh->is_canceling() || !rclcpp::ok() || std::chrono::steady_clock::now() > deadline)
        return false;
    }
    return true;
  };

  std::vector<GoalHandleTp::SharedPtr> handles;
  for (auto& f : goal_futures)
  {
    if (!wait(f) || !f.get())
    {
      error = "time parametrization goal rejected (or not answered)";
      for (auto& h : handles)
        time_parametrization_client_->async_cancel_goal(h);
      return false;
    }
    handles.push_back(f.get());
  }

  double t_offset = 0.0;
  for (size_t p = 0; p < handles.size(); p++)
  {
    publishFeedback(p, "retiming path " + std::to_string(pieces.at(p).index));
    auto result_future = time_parametrization_client_->async_get_result(handles.at(p));
    if (!wait(result_future))
    {
      error = gh->is_canceling() ? "canceled" : "time parametrization timeout";
      for (size_t q = p; q < handles.size(); q++)
        time_parametrization_client_->async_cancel_goal(handles.at(q));
      return false;
    }
    const std::string piece_name = "path " + std::to_string(pieces.at(p).index);
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
        wrapped.result->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      error = "time parametrization of " + piece_name + " failed (MoveIt error code " +
              std::to_string(wrapped.result ? wrapped.result->motion_plan_response.error_code.val : 0) + ")";
      return false;
    }
    const auto& piece_traj = wrapped.result->motion_plan_response.trajectory.joint_trajectory;
    if (piece_traj.points.empty())
    {
      error = "time parametrization of " + piece_name + " returned an empty trajectory";
      return false;
    }

    // The server may order the joints differently: map them to joint_names_.
    std::vector<size_t> perm(joint_names_.size());
    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      auto it = std::find(piece_traj.joint_names.begin(), piece_traj.joint_names.end(), joint_names_.at(j));
      if (it == piece_traj.joint_names.end())
      {
        error = "time parametrization returned a trajectory without joint " + joint_names_.at(j);
        return false;
      }
      perm.at(j) = static_cast<size_t>(it - piece_traj.joint_names.begin());
    }
    auto permute = [&perm](const std::vector<double>& v) {
      std::vector<double> out;
      if (v.empty())
        return out;
      out.reserve(perm.size());
      for (size_t idx : perm)
        out.push_back(idx < v.size() ? v.at(idx) : 0.0);
      return out;
    };

    // The first point of a piece is the last point of the previous one (both at rest): skip it.
    for (size_t k = (p == 0 ? 0 : 1); k < piece_traj.points.size(); k++)
    {
      const auto& src = piece_traj.points.at(k);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = permute(src.positions);
      point.velocities = permute(src.velocities);
      point.accelerations = permute(src.accelerations);
      point.time_from_start =
          rclcpp::Duration::from_seconds(t_offset + rclcpp::Duration(src.time_from_start).seconds());
      timed.points.push_back(point);
    }
    t_offset += rclcpp::Duration(piece_traj.points.back().time_from_start).seconds();
  }

  RCLCPP_INFO(node_->get_logger(), "compute_joint_trajectory: %zu paths, %zu points, %.2f s", pieces.size(),
              timed.points.size(), t_offset);
  return true;
}

void ComputeJointPathServer::publishDisplayTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                      const Eigen::VectorXd& q_start, bool timed)
{
  moveit_msgs::msg::DisplayTrajectory display;
  display.model_id = robot_model_->getName();
  display.trajectory_start.joint_state.name = joint_names_;
  display.trajectory_start.joint_state.position.assign(q_start.data(), q_start.data() + q_start.size());

  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = trajectory;
  if (!timed)
  {
    // Untimed path: it does not contain the current configuration, and the timing is fake
    // (display_time_step between consecutive points), only used by Rviz to animate it.
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = display.trajectory_start.joint_state.position;
    auto& points = robot_trajectory.joint_trajectory.points;
    points.insert(points.begin(), start);
    for (size_t k = 0; k < points.size(); k++)
      points.at(k).time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(k) * display_time_step_);
  }

  display.trajectory.push_back(robot_trajectory);
  display_pub_->publish(display);
}

void ComputeJointPathServer::execute(const std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  const size_t n = goal->poses.poses.size();
  auto result = std::make_shared<Action::Result>();
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->total_poses = static_cast<uint32_t>(n);

  auto abort = [&](const std::string& message) {
    RCLCPP_ERROR(node_->get_logger(), "compute_joint_path failed: %s", message.c_str());
    result->success = false;
    result->message = message;
    gh->abort(result);
  };
  auto publishFeedback = [&](size_t index, const std::string& status) {
    feedback->current_index = static_cast<uint32_t>(index);
    feedback->status = status;
    gh->publish_feedback(feedback);
  };

  std::string error;
  Eigen::VectorXd q_start;

  publishFeedback(0, "reading the robot configuration");
  if (!getCurrentConfiguration(q_start, error))
    return abort(error);

  publishFeedback(0, "updating the planning scene");
  if (!updatePlanningScene(error))
    return abort(error);

  publishFeedback(0, "setting the IK tool frame");
  if (!changeTool(goal->planning_frame, error))
    return abort(error);

  publishFeedback(0, "computing inverse kinematics");
  std::vector<Candidates> process, deviation;
  const auto ik_start = std::chrono::steady_clock::now();
  if (!solveIk(*goal, q_start, process, deviation, error))
    return abort(error);
  auto countSolved = [](const std::vector<Candidates>& c) {
    return std::count_if(c.begin(), c.end(), [](const Candidates& s) { return !s.empty(); });
  };
  RCLCPP_INFO(node_->get_logger(),
              "IK computed in %.2f s: %ld/%zu poses and %ld/%zu leave/approach poses have at least one solution",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - ik_start).count(), countSolved(process),
              n, countSolved(deviation), n);
  if (countSolved(process) == 0)
    return abort("no IK solution for any pose: check planning_frame orientation (its z axis must follow the "
                 "pose z axis) and that the poses are within reach");

  if (gh->is_canceling())
  {
    result->message = "canceled";
    gh->canceled(result);
    return;
  }

  std::vector<bool> segment_start(n, false);
  for (uint32_t idx : goal->jump_indices)
    segment_start.at(idx) = true;

  std::string planner_name;
  const MultigoalClient::SharedPtr multigoal_client = selectMultigoalClient(planner_name, error);
  if (!multigoal_client)
    return abort(error);
  RCLCPP_INFO(node_->get_logger(), "free motions planned by %s", planner_name.c_str());

  graph::core::CollisionCheckerPtr checker = std::make_shared<graph::collision_check::ParallelMoveitCollisionChecker>(
      planning_scene_, group_name_, graph_logger_, checker_threads_, checker_resolution_);

  auto planner = [this, gh, multigoal_client, planner_name](const Eigen::VectorXd& q_from,
                                                            const std::vector<Eigen::VectorXd>& goals,
                                                            std::vector<Eigen::VectorXd>& free_path,
                                                            size_t& goal_index) {
    return planMultigoal(multigoal_client, planner_name, q_from, goals, free_path, goal_index, gh);
  };
  JointPathBuilder builder(checker, lower_bound_, upper_bound_, node_->get_logger().get_child("builder"), planner,
                           builder_options_);
  JointPathBuilderResult path = builder.build(q_start, process, deviation, segment_start,
                                              [&](size_t index, const std::string& status) {
                                                if (gh->is_canceling())
                                                  return false;
                                                publishFeedback(index, status);
                                                return true;
                                              });

  if (gh->is_canceling())
  {
    result->message = "canceled";
    gh->canceled(result);
    return;
  }
  if (!path.success)
    return abort(path.message);

  result->trajectory.header.stamp = node_->now();
  result->trajectory.header.frame_id = goal->poses.header.frame_id;
  result->trajectory.joint_names = joint_names_;
  result->trajectory.points.reserve(path.waypoints.size());
  for (const Eigen::VectorXd& q : path.waypoints)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(q.data(), q.data() + q.size());
    result->trajectory.points.push_back(point);
  }
  result->pose_indices.assign(path.pose_indices.begin(), path.pose_indices.end());
  result->skipped_indices = path.skipped_indices;
  result->message = path.message;

  splitPath(q_start, path, result->paths, result->path_types);
  for (auto& p : result->paths)
    p.header = result->trajectory.header;

  RCLCPP_INFO(node_->get_logger(), "compute_joint_path: %zu points, %zu paths, %zu poses skipped",
              result->trajectory.points.size(), result->paths.size(), result->skipped_indices.size());

  result->success = true;
  publishDisplayTrajectory(result->trajectory, q_start, false);
  gh->succeed(result);
}

}  // namespace surface_path_planning
