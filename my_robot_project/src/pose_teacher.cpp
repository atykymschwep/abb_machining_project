#include "my_robot_project/pose_teacher.hpp"

PoseTeacher::PoseTeacher() : Node("pose_teacher") {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("pose_teacher_server", this);

    makeMarker();
    server_->applyChanges();
    
    RCLCPP_INFO(this->get_logger(), "Pose Teacher started. Move marker in RViz!");
}

void PoseTeacher::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
        Eigen::Affine3d pose;

        // quateritication
        Eigen::Quaterniond q(
            feedback->pose.orientation.w,
            feedback->pose.orientation.x,
            feedback->pose.orientation.y,
            feedback->pose.orientation.z);
        
        pose = Eigen::Translation3d(
            feedback->pose.position.x,
            feedback->pose.position.y,
            feedback->pose.position.z) * q;

        ikfast_abb::Solutions solutions = ikfast_abb::computeIK(pose);

        if (!solutions.empty()) {
            publishJoints(solutions[0]);
            last_valid_joints_ = solutions[0];
        }
    } 
    else if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
        RCLCPP_INFO(this->get_logger(), "-- Point is fixed --");
        printf("Angle joints (rad): [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n", 
               last_valid_joints_[0], last_valid_joints_[1], last_valid_joints_[2],
               last_valid_joints_[3], last_valid_joints_[4], last_valid_joints_[5]);
        RCLCPP_INFO(this->get_logger(), "data for T1, T3, Tvia");
    }
}

void PoseTeacher::makeMarker() {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = "ee_marker";
    int_marker.description = "Move to find T1/T3";
    int_marker.scale = 0.3;

    int_marker.pose.position.x = 0.9;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.5;

    auto makeControl = [](visualization_msgs::msg::InteractiveMarker &m, double w, double x, double y, double z, std::string name, bool move) {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation.w = w;
        control.orientation.x = x;
        control.orientation.y = y;
        control.orientation.z = z;
        control.name = (move ? "move_" : "rotate_") + name;
        control.interaction_mode = move ? 
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS : 
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        m.controls.push_back(control);
    };

    
    makeControl(int_marker, 1, 1, 0, 0, "x", true);
    makeControl(int_marker, 1, 1, 0, 0, "x", false);
    makeControl(int_marker, 1, 0, 1, 0, "z", true);
    makeControl(int_marker, 1, 0, 1, 0, "z", false);
    makeControl(int_marker, 1, 0, 0, 1, "y", true);
    makeControl(int_marker, 1, 0, 0, 1, "y", false);

    server_->insert(int_marker, std::bind(&PoseTeacher::processFeedback, this, std::placeholders::_1));
}

void PoseTeacher::publishJoints(const ikfast_abb::JointValues &q) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    js.position.assign(q.begin(), q.end());
    joint_pub_->publish(js);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseTeacher>());
    rclcpp::shutdown();
    return 0;
}