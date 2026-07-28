#include "surface_path_planning/robot_description_node.hpp"

namespace surface_path_planning {

    RobotDescriptionNode::RobotDescriptionNode() : Node("robot_description_node")
    {
        RCLCPP_INFO(this->get_logger(), "Started RobotDescriptionNode, listening to /robot_description...");

        // Qui usiamo il tipo base <std_msgs::msg::String>
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&RobotDescriptionNode::callback, this, std::placeholders::_1));
    }

    void RobotDescriptionNode::callback(const std_msgs::msg::String::SharedPtr msg)
    {
        robot_description_ = msg->data;
        std::string truncated = robot_description_.substr(0, 200);
        RCLCPP_INFO(this->get_logger(), "Robot description payload cached. Preview:\n%s...", truncated.c_str());
    }

    bool RobotDescriptionNode::isRobotDescriptionReceived() const {
        return !robot_description_.empty();
    }

    std::string RobotDescriptionNode::getRobotDescription() const {
        return robot_description_;
    }

} // namespace surface_path_planning