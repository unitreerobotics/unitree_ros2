/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

enum TestMode {
  /*---Basic motion---*/
  NORMAL_STAND,
  BALANCE_STAND,
  VELOCITY_MOVE,
  STAND_DOWN,
  STAND_UP,
  DAMP,
  RECOVERY_STAND,
  /*---Special motion ---*/
  SIT,
  RISE_SIT,
  MOVE,
  STOP_MOVE,
  
};

class Go2SportClientNode : public rclcpp::Node {
 public:
  explicit Go2SportClientNode(int test_mode)
      : Node("go2_sport_client_node"),
        sport_client_(this),
        test_mode_(test_mode) {
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data);
        });
    t1_ = std::thread([this] {
      // wait for ros2 spin
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      RobotControl();
    });
  }

  void RobotControl() {
    ct_ += dt_;
    switch (test_mode_) {
      case NORMAL_STAND:
        sport_client_.StandUp(req_);
        break;
      case BALANCE_STAND:
        sport_client_.BalanceStand(req_);
        break;
      case VELOCITY_MOVE:
        sport_client_.Move(req_, 0.3, 0, 0.3);
        break;
      case STAND_DOWN:
        sport_client_.StandDown(req_);
        break;
      case STAND_UP:
        sport_client_.StandUp(req_);
        break;
      case DAMP:
        sport_client_.Damp(req_);
        break;
      case RECOVERY_STAND:
        sport_client_.RecoveryStand(req_);
        break;
      case SIT:
        if (flag_ == 0) {
          sport_client_.Sit(req_);
          flag_ = 1;
        }
        break;
      case RISE_SIT:
        if (flag_ == 0) {
          sport_client_.RiseSit(req_);
          flag_ = 1;
        }
        break;
      case MOVE:
        sport_client_.Move(req_, 0.3, 0, 0);
        break;
      case STOP_MOVE:
        sport_client_.StopMove(req_);
        break;
      default:
        sport_client_.StopMove(req_);
    }
  }

  void GetInitState() {
    px0_ = state_.position[0];
    py0_ = state_.position[1];
    yaw0_ = state_.imu_state.rpy[2];
    RCLCPP_INFO(this->get_logger(),
                "initial position: x0: %f, y0: %f, yaw0: %f", px0_, py0_,
                yaw0_);
  }

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
    state_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f", state_.position[0],
                state_.position[1], state_.position[2]);
    RCLCPP_INFO(this->get_logger(), "IMU rpy: %f, %f, %f",
                state_.imu_state.rpy[0], state_.imu_state.rpy[1],
                state_.imu_state.rpy[2]);
  }

 private:
  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;

  rclcpp::TimerBase::SharedPtr timer_;
  unitree_api::msg::Request req_;  // Unitree Go2 ROS2 request message
  double px0_{}, py0_{}, yaw0_{};
  double ct_{};
  int flag_{};
  float dt_ = 0.1;
  int test_mode_;
  std::thread t1_;
};

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <test_mode>" << std::endl;
    std::cerr << "Available test modes:" << std::endl;
    std::cerr << "  0: NORMAL_STAND" << std::endl;
    std::cerr << "  1: BALANCE_STAND" << std::endl;
    std::cerr << "  2: VELOCITY_MOVE" << std::endl;
    std::cerr << "  3: STAND_DOWN" << std::endl;
    std::cerr << "  4: STAND_UP" << std::endl;
    std::cerr << "  5: DAMP" << std::endl;
    std::cerr << "  6: RECOVERY_STAND" << std::endl;
    std::cerr << "  7: SIT" << std::endl;
    std::cerr << "  8: RISE_SIT" << std::endl;
    std::cerr << "  9: MOVE" << std::endl;
    std::cerr << "  10: STOP_MOVE" << std::endl;
    return 1;
  }

  int test_mode = std::atoi(argv[1]); // NOLINT

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2SportClientNode>(test_mode);
  node->GetInitState();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}