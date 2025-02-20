#include <signal.h>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>


using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavToPoseClient : public rclcpp::Node {

public:

    NavToPoseClient()
        : Node("nav_to_pose_client"),
          action_client_(rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose")) {

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
        RCLCPP_INFO(this->get_logger(), "NavToPoseClient Node Initialized!");
        
        // Handle SIGINT (Ctrl+C)
        signal(SIGINT, sigint_handler);
    }

    void send_goal(double x, double y, double yaw) {
        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        auto q = yaw_to_quaternion(yaw);
        goal_msg.pose.pose.orientation.z = q[2];
        goal_msg.pose.pose.orientation.w = q[3];

        RCLCPP_INFO(this->get_logger(), "Sending Goal: [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);
        publish_goal_marker(goal_msg.pose.pose);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavToPoseClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavToPoseClient::get_result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    static void sigint_handler(int) {
        rclcpp::shutdown();
    }

    std::vector<double> yaw_to_quaternion(double yaw) {
        return {0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)};
    }

    void goal_response_callback(GoalHandleNav::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted!");
        }
    }

    void get_result_callback(const GoalHandleNav::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            delete_goal_marker();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed.");
        }
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

int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();

    if (argc < 4) {
        RCLCPP_ERROR(node->get_logger(), "Usage: et_nav_goal x=<value> y=<value> yaw=<value>");
        return 1;
    }

    double x =0.0, y = 0.0, yaw = 0.0;
    for( int i =1; i<argc; i++)
    {
        std::string arg = argv[i];
        if(arg.rfind("x=",0) == 0)
        {
            x = std::stod(arg.substr(2));
        }else if(arg.rfind("y=",0) == 0)
        {
            y = std::stod(arg.substr(2));  
        }else if (arg.rfind("yaw=",0) == 0)
        {
            yaw = std::stod(arg.substr(4)); 
        }else
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid argument: %s. Use x=<value> y=<value> yaw=<value>", arg.c_str());
            return 1;  
        }
    }

    RCLCPP_INFO(node->get_logger(), "Received Goal Input: x = %.2f, y = %.2f, yaw = %.2f", x, y, yaw);

    node->send_goal(x, y, yaw);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
