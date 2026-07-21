#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <string>

class ModelSpawner : public rclcpp::Node {
public:
    ModelSpawner();

private:
    void publish_marker(const std::string & file_path);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string file_path_;
};
