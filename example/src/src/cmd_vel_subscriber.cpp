/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
/**
 * This example demonstrates how to subscribe to standard ROS2 cmd_vel topic
 * and convert it to Unitree robot motion commands
 **/
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

#include "common/ros2_sport_client.h"

class CmdVelSubscriber : public rclcpp::Node {
 public:
  CmdVelSubscriber() : Node("cmd_vel_subscriber") {
    // Subscribe to standard cmd_vel topic
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmdVelCallback(msg);
        });

    // Create sport client for sending motion commands
    sport_client_ = std::make_unique<SportClient>(this);

    RCLCPP_INFO(this->get_logger(),
                "CmdVel subscriber initialized. Listening to /cmd_vel topic.");
  }

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract velocity components from Twist message
    // For mobile robots: linear.x = forward velocity, angular.z = rotation
    float vx = static_cast<float>(msg->linear.x);
    float vy = static_cast<float>(msg->linear.y);
    float vyaw = static_cast<float>(msg->angular.z);

    // Log received velocities
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Received cmd_vel: vx=%.3f, vy=%.3f, vyaw=%.3f", vx, vy, vyaw);

    // Convert to Unitree motion command
    // Create request message
    unitree_api::msg::Request req;
    
    // Send velocity command using SportClient
    sport_client_->Move(req, vx, vy, vyaw);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  std::unique_ptr<SportClient> sport_client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

