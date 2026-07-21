#include "my_robot_project/model_spawner.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

ModelSpawner::ModelSpawner() : Node("model_spawner") {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    
    // path
    std::string pkg_share_path = ament_index_cpp::get_package_share_directory("my_robot_project");
    file_path_ = pkg_share_path + "/meshes/truck.stl";

    // check file
    std::ifstream f(file_path_.c_str());
    if (f.good()) {
        RCLCPP_INFO(this->get_logger(), "File found! Loading: %s", file_path_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "FILE NOT FOUND at: %s. Check the path!", file_path_.c_str());
    }

    // publish every 2 sec model
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        [this]() { this->publish_marker(file_path_); }
    );
}

void ModelSpawner::publish_marker(const std::string & file_path) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "truck";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.mesh_resource = "file://" + file_path;

    // position in the map
    marker.pose.position.x = 1.0; 
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.5;
    marker.pose.orientation.w = 1.0;

    // scales
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // colors (gray)
    marker.color.r = 0.6f;
    marker.color.g = 0.6f;
    marker.color.b = 0.6f;
    marker.color.a = 1.0f;

    publisher_->publish(marker);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelSpawner>());
    rclcpp::shutdown();
    return 0;
}