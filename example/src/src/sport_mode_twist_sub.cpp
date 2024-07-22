#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

class MovePublisher : public rclcpp::Node
{
public:
    MovePublisher() : Node("move_wrapper")
    {
        publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    }

    // Method to move the robot
    void moverobot(float x, float y, float yaw)
    {
        SportClient sport_req; // SportClient instance
        unitree_api::msg::Request req;
        sport_req.Move(req, x, y, yaw); // Create a move request
        publisher_->publish(req);       // Publish the request
    }

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
};

class TwistSubscriber : public rclcpp::Node
{
public:
    TwistSubscriber(std::shared_ptr<MovePublisher> move_pub) : Node("twist_subscriber"), move_publisher_(move_pub)
    {
        this->declare_parameter<std::string>("twist_topic", "cmd_vel");
        std::string twist_topic = this->get_parameter("twist_topic").as_string();

        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_topic, 10, std::bind(&TwistSubscriber::twist_callback, this, _1));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    std::shared_ptr<MovePublisher> move_publisher_;

    // Callback function for receiving twist messages
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "Received twist - lx:'%f', ly: '%f', az:'%f'", message->linear.x, message->linear.y, message->angular.z);

        // Forward the twist message values to the MovePublisher
        move_publisher_->moverobot(message->linear.x, message->linear.y, message->angular.z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto move_publisher = std::make_shared<MovePublisher>();
    auto twist_subscriber = std::make_shared<TwistSubscriber>(move_publisher);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(twist_subscriber);
    executor.add_node(move_publisher);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

