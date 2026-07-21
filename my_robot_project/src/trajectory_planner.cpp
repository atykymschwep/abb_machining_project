#include "my_robot_project/trajectory_planner.hpp"
#include <cmath>

TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner") {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_path", 10);
    
    // init points
    T1 = {0.943436, 0.616708, 0.599644, -0.658087, 1.855479, 2.930360};
    T3 = {0.910996, 0.905774, 0.525225, -0.651910, 1.629859, 3.048407};
    Tvia = {0.734916, 1.170104, -0.394137, -0.918181, 2.126425, -0.539421};
    Home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    T2 = calculateRelativePoint(T1, 0.0, 0.6, 0.0);
    T4 = calculateRelativePoint(T3, 0.0, 0.6, 0.0);

    idle_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryPlanner::publishIdleState, this));

    service_ = this->create_service<std_srvs::srv::Trigger>("execute_machining",std::bind(&TrajectoryPlanner::handleExecuteService, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to work. Wait a call /execute_machining.");
}

void TrajectoryPlanner::publishIdleState() {
    if (!is_moving_) {
        publishJoints(Home, 0.1);
    }; 
}

void TrajectoryPlanner::handleExecuteService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if (is_moving_) {
        response->success = false;
        response->message = "Robot is already moving!";
        return;
    }

    is_moving_ = true;
    marker_id_ = 0; 
    prev_q_ = Home;
    prev_v_ = {0,0,0,0,0,0};

    JointValues T1_app = calculateRelativePoint(T1, 0.0, 0.0, -0.2); 
    JointValues T2_app = calculateRelativePoint(T2, 0.0, 0.0, -0.2);
    JointValues T3_app = calculateRelativePoint(T3, 0.0, 0.0, -0.2);
    JointValues T4_app = calculateRelativePoint(T4, 0.0, 0.0, -0.2);

    movePTP(Home, T1_app, 2.0, 0.0, 0.0, 1.0); 
    moveLIN(T1_app, T1, 1.0, 1.0, 0.0, 0.0);    
    moveLIN(T1, T2, 4.0, 0.0, 1.0, 0.0);        
    moveLIN(T2, T2_app, 1.0, 1.0, 0.0, 0.0);
    movePTP(T2_app, Tvia, 2.0, 0.0, 0.0, 1.0);    

    movePTP(Tvia, T4_app, 2.0, 0.0, 0.0, 1.0);   

    moveLIN(T4_app, T4, 1.0, 1.0, 0.0, 0.0);    
    moveLIN(T4, T3, 4.0, 0.0, 1.0, 0.0);        
    moveLIN(T3, T3_app, 1.0, 1.0, 0.0, 0.0);    

    movePTP(T3_app, Home, 2.0, 0.0, 0.0, 1.0);   

    is_moving_ = false;
    response->success = true;
    response->message = "Loop is finished!";
}

void TrajectoryPlanner::movePTP(const JointValues& start, const JointValues& target, double duration, float r, float g, float b) {
    // setting of steps
    int steps = 100;
    double dt = duration / steps;
    rclcpp::Rate rate(1.0 / dt);
    auto current_line = createLineMarker(r, g, b);

    for (int i = 0; i <= steps; ++i) {
        // progress
        double t = (double)i / steps;
        // smooth move
        double s = t - (std::sin(2.0 * M_PI * t) / (2.0 * M_PI)); 

        // calc joints
        JointValues current;
        for (int j = 0; j < 6; ++j) {
            current[j] = start[j] + (target[j] - start[j]) * s;
        }

        // give coord
        Eigen::Affine3d pose = computeFk(current);
        geometry_msgs::msg::Point pt;
        pt.x = pose.translation().x();
        pt.y = pose.translation().y();
        pt.z = pose.translation().z();

        // color line
        current_line.points.push_back(pt);
        current_line.header.stamp = this->now();
        marker_pub_->publish(current_line);
        
        publishJoints(current, dt);
        rate.sleep();
    }
}

void TrajectoryPlanner::moveLIN(const JointValues& start_q, const JointValues& target_q, double duration, float r, float g, float b) {
    // get matrix with coord and angles
    Eigen::Affine3d start_pose = computeFk(start_q);
    Eigen::Affine3d target_pose = computeFk(target_q);
    // setting of steps
    int steps = 100;
    double dt = duration / steps;
    rclcpp::Rate rate(1.0 / dt);
    auto current_line = createLineMarker(r, g, b);

    for (int i = 0; i <= steps; ++i) {
        // progress
        double t = (double)i / steps;
        // smooth move
        double s = t - (std::sin(2.0 * M_PI * t) / (2.0 * M_PI));
        //coord of line, rotations and combine it
        Eigen::Vector3d pos = start_pose.translation() + s * (target_pose.translation() - start_pose.translation());
        Eigen::Quaterniond q_interp = Eigen::Quaterniond(start_pose.rotation()).slerp(s, Eigen::Quaterniond(target_pose.rotation()));
        Eigen::Affine3d current_pose = Eigen::Translation3d(pos) * q_interp;
        
        // give coord
        geometry_msgs::msg::Point pt;
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        // color line
        current_line.points.push_back(pt);
        current_line.header.stamp = this->now();
        marker_pub_->publish(current_line);

        // publish first sol
        Solutions solutions = computeIK(current_pose);
        if (!solutions.empty()) {
            publishJoints(solutions[0], dt);
        }
        rate.sleep();
    }
}

visualization_msgs::msg::Marker TrajectoryPlanner::createLineMarker(float r, float g, float b) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "base_link";
    line.ns = "robot_path";
    line.id = marker_id_++; 
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.scale.x = 0.005; 
    line.color.r = r;
    line.color.g = g; 
    line.color.b = b;
    line.color.a = 1.0;
    return line;
}

JointValues TrajectoryPlanner::calculateRelativePoint(const JointValues& base_q, double dx, double dy, double dz) {
    Eigen::Affine3d pose = computeFk(base_q);
    pose.translate(Eigen::Vector3d(dx, dy, dz));
    Solutions sol = computeIK(pose);
    return sol.empty() ? base_q : sol[0];
}

void TrajectoryPlanner::publishJoints(const JointValues& q, double dt) {
    // setting
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    
    // veсtors of vel and acc
    std::vector<double> v(6, 0.0), a(6, 0.0);
    // calc vels and accs
    if (is_moving_ && dt > 0) {
        for(int i=0; i<6; i++) {
            v[i] = (q[i] - prev_q_[i]) / dt;
            a[i] = (v[i] - prev_v_[i]) / dt;
        }
    }
    // assign values
    js.position.assign(q.begin(), q.end());
    js.velocity = v;
    js.effort = a;

    prev_q_ = q;
    for (int i = 0; i < 6; ++i) {
        prev_v_[i] = v[i];
    }

    joint_pub_->publish(js);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}