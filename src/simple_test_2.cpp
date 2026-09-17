#include <rclcpp/rclcpp.hpp>
#include "surface_path_planning/robot_description_node.hpp"
#include "surface_path_planning/surface_path_planning.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Initialize the robot description node and wait for the robot description to be received.
  auto robot_description_node = std::make_shared<surface_path_planning::RobotDescriptionNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(robot_description_node);

  auto start_time = std::chrono::steady_clock::now();
  std::chrono::seconds timeout_duration(30);

  // wait for robot description (timeout 30 seconds)
  while (rclcpp::ok() && !robot_description_node->isRobotDescriptionReceived()) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if ((std::chrono::steady_clock::now() - start_time) > timeout_duration) {
      RCLCPP_ERROR(robot_description_node->get_logger(), "Fatal timeout tracking down critical /robot_description context. Exiting.");
      return 1;
    }
  }

  std::string urdf_payload = robot_description_node->getRobotDescription();
  robot_description_node.reset();

  // planner is a shared_ptr to surface_path_planning::SurfacePlannerNode
  auto planner_node = std::make_shared<surface_path_planning::SurfacePlannerNode>();

  // prima il plan poi l'execute, dopo conferma utente y/n
  RCLCPP_INFO(planner_node->get_logger(), "Planning pipeline execution started.");
  if (planner_node->executePlanningCartesian(urdf_payload)) {
    RCLCPP_INFO(planner_node->get_logger(), "Planning pipeline execution finished. Keeping node spin active for visualization persistence.");
    rclcpp::spin(planner_node);
  }

  rclcpp::shutdown();
  return 0;
}