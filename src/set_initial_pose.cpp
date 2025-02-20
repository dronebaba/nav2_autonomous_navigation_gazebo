#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>

class AutonomousNavigation : public rclcpp::Node {
    
public:

    AutonomousNavigation() : Node("autonomous_navigation") {
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    }

    void set_initial_pose(const geometry_msgs::msg::Pose &pose) {
        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.header.stamp = this->get_clock()->now();
        initial_pose_msg.pose.pose = pose;
        
        initial_pose_publisher_->publish(initial_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose");
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

};

geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw) {

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousNavigation>();
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;
    initial_pose.position.y = 0.0; // 2.0
    initial_pose.position.z = 0.0;
    
    double yaw = 0.0;
    initial_pose.orientation = quaternion_from_euler(0.0, 0.0, yaw);
    
    node->set_initial_pose(initial_pose);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
