#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace surface_path_planning {

    /**
     * @brief Helper node designed to latch onto the "/robot_description" topic.
     * Useful to block startup sequences until the URDF is fully published and available.
     */
    class RobotDescriptionNode : public rclcpp::Node
    {
        using Ptr = std::shared_ptr<RobotDescriptionNode>;
    public:
        RobotDescriptionNode();

        bool isRobotDescriptionReceived() const;
        std::string getRobotDescription() const;

    private:
        void callback(const std_msgs::msg::String::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::string robot_description_ = "";
    };

} // namespace surface_path_planning