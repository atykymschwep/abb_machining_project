#pragma once

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>
#include <memory>
#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

class PoseTeacher : public rclcpp::Node {
public:
    PoseTeacher();

private:
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void makeMarker();
    void publishJoints(const ikfast_abb::JointValues &q);

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    ikfast_abb::JointValues last_valid_joints_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};