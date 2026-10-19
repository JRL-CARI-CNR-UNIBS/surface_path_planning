// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Authors: Manuel Beschi <manuel.beschi@unibs.it>, Elisa Zappa

#include <rclcpp/rclcpp.hpp>

#include "surface_path_planning/compute_joint_path_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("compute_joint_path");

  auto server = std::make_shared<surface_path_planning::ComputeJointPathServer>(node);
  // The robot model is loaded before the node is added to the executor.
  if (!server->init())
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
