/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 NOTICE: This client is only available on ROS2 Humble.
***********************************************************************/
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#include "common/ros2_robot_state_client.h"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

class Go2RobotStateClientNode : public rclcpp::Node {
 public:
  explicit Go2RobotStateClientNode()
      : Node("go2_robot_state_client_node"), robot_state_client_(this) {
    t1_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      RobotControl();
    });
  }

  void RobotControl() {
    int32_t status = 0;
    int32_t ret = robot_state_client_.SetReportFreq(3, 30);
    std::cout << "Call SetReportFreq[3,30] ret:" << ret << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    const auto *serviceName = "sport_mode";
    ret = robot_state_client_.ServiceSwitch(serviceName, 0, status);
    std::cout << "Call ServiceSwitch[" << serviceName << ",0] ret:" << ret
              << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    ret = robot_state_client_.ServiceSwitch(serviceName, 1, status);
    std::cout << "Call ServiceSwitch[" << serviceName << ",1] ret:" << ret
              << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::vector<ServiceState> serviceStateList;
    ret = robot_state_client_.ServiceList(serviceStateList);
    std::cout << "Call ServiceList ret:" << ret << std::endl;

    size_t i = 0;
    size_t count = serviceStateList.size();
    std::cout << "serviceStateList size:" << count << std::endl;

    for (i = 0; i < count; i++) {
      const ServiceState &serviceState = serviceStateList[i];
      std::cout << "name:" << serviceState.name
                << ", status:" << serviceState.status
                << ", protect:" << serviceState.protect << std::endl;
    }
  }

 private:
  RobotStateClient robot_state_client_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;
  unitree_api::msg::Request req_;
  std::thread t1_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2RobotStateClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}