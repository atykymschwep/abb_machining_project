#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Geometry>
#include <vector>
#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

using namespace ikfast_abb;

class TrajectoryPlanner : public rclcpp::Node {
public:
    TrajectoryPlanner();

private:
    // handle events
    void publishIdleState();
    void handleExecuteService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // move methods
    void movePTP(const JointValues& start, const JointValues& target, double duration, float r, float g, float b);
    void moveLIN(const JointValues& start_q, const JointValues& target_q, double duration, float r, float g, float b);

    // additional func
    visualization_msgs::msg::Marker createLineMarker(float r, float g, float b);
    JointValues calculateRelativePoint(const JointValues& base_q, double dx, double dy, double dz);
    void publishJoints(const JointValues& q, double dt);

    // pub and ser
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr idle_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // points and states
    bool is_moving_ = false;
    int marker_id_ = 0;
    JointValues prev_q_ = {0,0,0,0,0,0};
    JointValues prev_v_ = {0,0,0,0,0,0};
    JointValues T1, T2, T3, T4, Tvia, Home;
};
