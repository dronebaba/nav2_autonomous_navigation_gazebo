#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <memory>

class WaypointClient : public rclcpp::Node {

public:

    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    WaypointClient() : Node("waypoint_client") {
        action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
        current_waypoint_index_ = 0;
    }

    void send_waypoints(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints) {
        waypoints_ = waypoints;
        send_next_waypoint();
    }

private:

    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    size_t current_waypoint_index_;

    void send_next_waypoint() {
        if (current_waypoint_index_ < waypoints_.size()) {

            auto goal_msg = FollowWaypoints::Goal();
            goal_msg.poses.push_back(waypoints_[current_waypoint_index_]);

            if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) 
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available!");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Sending waypoint: [%f, %f]", 
                    waypoints_[current_waypoint_index_].pose.position.x, 
                    waypoints_[current_waypoint_index_].pose.position.y);

            publish_goal_marker(waypoints_[current_waypoint_index_].pose);

            rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
            options.goal_response_callback = std::bind(&WaypointClient::goal_response_callback, this, std::placeholders::_1);
            options.result_callback = std::bind(&WaypointClient::get_result_callback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, options);
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "All waypoints visited !!!");
            delete_goal_marker();
        }
    }

    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr &goal_handle) {

        if (!goal_handle) {

            RCLCPP_WARN(this->get_logger(), "Goal rejected!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted!");
    }

    void get_result_callback(const GoalHandleFollowWaypoints::WrappedResult &result) {

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {

            RCLCPP_INFO(this->get_logger(), "Waypoint reached!");
            RCLCPP_INFO(this->get_logger(), "Waypoint reached, waiting for 5 seconds ...");
            rclcpp::sleep_for(std::chrono::seconds(5));

        } else {
            RCLCPP_WARN(this->get_logger(), "Goal failed or was canceled.");
        }

        // Move to next waypoint
        current_waypoint_index_++;
        send_next_waypoint();
    }

    void publish_goal_marker(const geometry_msgs::msg::Pose &pose) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Goal Marker Published!");
    }

    void delete_goal_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Goal Marker Deleted!");
    }
};

// Convert yaw (Z-axis rotation) to quaternion
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
    return q;
}

// Helper function to create a PoseStamped waypoint
geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = yaw_to_quaternion(yaw);
    return pose;
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointClient>();

    // Add or delete the waypoints
    std::vector<geometry_msgs::msg::PoseStamped> waypoints = {
        create_pose(6.6, 6.6, 0), // waypoint_1 
        create_pose(4.4, 4.4, 0), // waypoint_2
        create_pose(6.6, 6.6, 0)  // waypoint_3
    };

    node->send_waypoints(waypoints);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
