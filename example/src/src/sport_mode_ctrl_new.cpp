#include <unistd.h>
#include <cmath>
#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono;

class MovePublisher : public rclcpp::Node
{
public:
    MovePublisher() : Node("move_sender")
    {
        publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    };

    // Method to move the robot
    void moverobot(float x, float y, float yaw)
    {
        SportClient sport_req; // SportClient instance
        unitree_api::msg::Request req;
        sport_req.Move(req, x, y, yaw); // Create a move request
        publisher_->publish(req);       // Publish the request
    }

    // Method to stop the robot
    void stoprobot()
    {
        SportClient sport_req; // SportClient instance
        unitree_api::msg::Request req;
        sport_req.StopMove(req);  // Create a stop request
        publisher_->publish(req); // Publish the request
    }

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
};

class TwistSubscriber : public rclcpp::Node
{
public:
    double requested_x = 0;
    double requested_y = 0;
    double requested_yaw = 0;
    bool requested_move = false;

    TwistSubscriber() : Node("twist_subscriber")
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "pythontopic", 10, std::bind(&TwistSubscriber::twist_callback, this, _1));
    };

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

    // Callback function for receiving twist messages
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "Request lx:'%f' ly: '%f' lz:'%f'", message->linear.x, message->linear.y, message->linear.z);
        RCLCPP_INFO(this->get_logger(), "Request ax:'%f' ay: '%f' az:'%f'", message->angular.x, message->angular.y, message->angular.z);

        requested_move = true;
        requested_x = message->linear.x;
        requested_y = message->linear.y;
        requested_yaw = message->angular.z;
    };
};

class StateSubscriber : public rclcpp::Node
{
public:
    StateSubscriber() : Node("state_subscriber")
    {
        subscriber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&StateSubscriber::state_callback, this, _1));
    };

    double px = 0;
    double py = 0;
    double yaw = 0;

private:
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscriber_;

    // Callback function for receiving state updates
    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        px = data->position[0];
        py = data->position[1];
        yaw = data->imu_state.rpy[2];
    };
};

class MoveHandler : public rclcpp::Node
{
public:
    MoveHandler() : Node("move_handler")
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1)), std::bind(&MoveHandler::timercallback, this));
    }

    void setnodes(std::shared_ptr<TwistSubscriber> twist_sub, std::shared_ptr<MovePublisher> move_pub, std::shared_ptr<StateSubscriber> state_sub)
    {
        twist_subscriber = twist_sub;
        move_publisher = move_pub;
        state_subscriber = state_sub;
    };

    void printdata()
    {
        std::cout << twist_subscriber->requested_x << std::endl;
    };

    // Timer callback to process movement commands
    void timercallback()
    {
        current_x = state_subscriber->px;
        current_y = state_subscriber->py;
        current_yaw = state_subscriber->yaw;

        double yaw_request_modifier = 1;
        double speed_X = 0.8;
        double speed_Y = 0.3;
        double speed_YAW = 2;

        double move_x = 0;
        double move_y = 0;
        double move_yaw = 0;

        double x_tolerance = 0.05;
        double y_tolerance = 0.03;
        double yaw_tolerance = 0.04;

        if (twist_subscriber->requested_move)
        {
            initial_x = state_subscriber->px;
            initial_y = state_subscriber->py;
            initial_yaw = state_subscriber->yaw;

            requested_x = twist_subscriber->requested_x;
            requested_y = twist_subscriber->requested_y;

            requested_yaw = fmod(twist_subscriber->requested_yaw, 2 * M_PI) * yaw_request_modifier;

            twist_subscriber->requested_move = false;
            reached_goal = false;
            reached_yaw = false;

            final_x = initial_x + (requested_x)*cos(initial_yaw);
            final_y = initial_y + (requested_x)*sin(initial_yaw);
            final_yaw = normalize_angle(initial_yaw + requested_yaw); // Normalize final_yaw to be within -π to π

            RCLCPP_INFO(this->get_logger(), "COS: %f SIN: %f", cos(initial_yaw), sin(initial_yaw));
            RCLCPP_INFO(this->get_logger(), "X: %f Y: %f YAW: %f", initial_x, initial_y, initial_yaw);
            RCLCPP_INFO(this->get_logger(), "X: %f Y: %f YAW: %f", final_x, final_y, final_yaw);
        }

        if (!reached_goal)
        {
            double target_distance = sqrt(pow((final_x - initial_x), 2) + pow((final_y - initial_y), 2));
            double current_distance = sqrt(pow((current_x - initial_x), 2) + pow((current_y - initial_y), 2));

            double diff_yaw = normalize_angle(final_yaw - current_yaw);
            double diff_distance = target_distance - current_distance;

            double y_error_a = final_y - initial_y;
            double y_error_b = -(final_x - initial_x);
            double y_error_c = (final_x - initial_x) * initial_y - (final_y - initial_y) * initial_x;

            double y_error_direction = y_error_a * current_x + y_error_b * current_y + y_error_c;
            double y_error_denominator = sqrt(pow(y_error_a, 2) + pow(y_error_b, 2));
            double y_error = (y_error_denominator != 0) ? abs(y_error_direction) / y_error_denominator : 0;

            speed_X = std::min(std::max(abs(diff_distance) - x_tolerance, 0.15), speed_X);
            speed_Y = std::min(std::max((abs(y_error) - y_tolerance) * 0.75, 0.05), speed_Y);
            speed_YAW = std::min(std::max(abs(diff_yaw) - yaw_tolerance, 0.5), speed_YAW);

            RCLCPP_INFO(this->get_logger(), "X | T: %f | C: %f | D: %f", target_distance, current_distance, diff_distance);
            // RCLCPP_INFO(this->get_logger(), "Y | I: %f | C: %f | F: %f | D: %f", initial_y, current_y, final_y, y_error);
            // RCLCPP_INFO(this->get_logger(), "Z | I: %f | C: %f | F: %f | D: %f", initial_yaw, current_yaw, final_yaw, diff_yaw);

            if (requested_x != 0)
            {

                if (abs(diff_distance) > x_tolerance)
                {

                    speed_X *= ((diff_distance > 0) - (diff_distance < 0));
                    move_x = speed_X;
                }

                if (abs(y_error) > y_tolerance)
                {
                    speed_Y *= (y_error > 0) - (y_error < 0);
                    move_y = speed_Y;
                }

                if (abs(diff_distance) < x_tolerance)
                {
                    reached_goal = true;
                    move_publisher->stoprobot();
                    RCLCPP_INFO(this->get_logger(), "---Destination Reached---");
                }
                else
                {
                    move_publisher->moverobot(move_x, move_y, move_yaw);
                }
            }

            else if (requested_yaw != 0)
            {
                if (abs(diff_yaw) > yaw_tolerance)
                {
                    // RCLCPP_INFO(this->get_logger(), "NOT WITHIN TOLERANCE");

                    if (requested_yaw == 0 || reached_yaw)
                    {
                        speed_YAW *= (diff_yaw > 0) - (diff_yaw < 0);
                    }
                    else
                    {
                        speed_YAW *= (requested_yaw > 0) - (requested_yaw < 0);
                    }
                    move_yaw = speed_YAW;
                }

                if (abs(diff_yaw) < yaw_tolerance)
                {
                    reached_goal = true;
                    move_publisher->stoprobot();
                    RCLCPP_INFO(this->get_logger(), "---Destination Reached---");
                }
                else
                {
                    move_publisher->moverobot(move_x, move_y, move_yaw);
                }
            }

            RCLCPP_INFO(this->get_logger(), "MX: %f | MY: %f | MZ: %f", move_x, move_y, move_yaw);
        }
    }

private:
    std::shared_ptr<TwistSubscriber> twist_subscriber;
    std::shared_ptr<MovePublisher> move_publisher;
    std::shared_ptr<StateSubscriber> state_subscriber;
    rclcpp::TimerBase::SharedPtr timer_;

    double initial_x = 0;
    double initial_y = 0;
    double initial_yaw = 0;

    double requested_x = 0;
    double requested_y = 0;
    double requested_yaw = 0;

    double final_x = 0;
    double final_y = 0;
    double final_yaw = 0;

    double current_x = 0;
    double current_y = 0;
    double current_yaw = 0;

    bool reached_goal = true;
    bool reached_yaw = false;

    // Helper function to normalize angles to -π to π range
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto twist_subscriber = std::make_shared<TwistSubscriber>();
    auto move_publisher = std::make_shared<MovePublisher>();
    auto state_subscriber = std::make_shared<StateSubscriber>();
    auto move_handler = std::make_shared<MoveHandler>();

    move_handler->setnodes(twist_subscriber, move_publisher, state_subscriber);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(twist_subscriber);
    executor.add_node(move_publisher);
    executor.add_node(state_subscriber);
    executor.add_node(move_handler);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
