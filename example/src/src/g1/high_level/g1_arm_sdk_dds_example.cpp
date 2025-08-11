#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "g1/g1.hpp"

// G1ARM5 or G1ARM7
#define G1ARM5 5
#define G1ARM7 7
#define ARM_TYPE G1ARM5

using namespace std::chrono_literals;
using LowCmd = unitree_hg::msg::LowCmd;
using LowState = unitree_hg::msg::LowState;

class ArmLowLevelController : public rclcpp::Node {
#if ARM_TYPE == G1ARM5
  static constexpr int NUM_ARM_JOINTS = 13;
  static constexpr auto NOT_USED_JOINT = G1Arm5JointIndex::NOT_USED_JOINT;
  std::array<G1Arm5JointIndex, NUM_ARM_JOINTS> arm_joints_ = {
      G1Arm5JointIndex::LEFT_SHOULDER_PITCH,
      G1Arm5JointIndex::LEFT_SHOULDER_ROLL,
      G1Arm5JointIndex::LEFT_SHOULDER_YAW,
      G1Arm5JointIndex::LEFT_ELBOW_PITCH,
      G1Arm5JointIndex::LEFT_ELBOW_ROLL,
      G1Arm5JointIndex::RIGHT_SHOULDER_PITCH,
      G1Arm5JointIndex::RIGHT_SHOULDER_ROLL,
      G1Arm5JointIndex::RIGHT_SHOULDER_YAW,
      G1Arm5JointIndex::RIGHT_ELBOW_PITCH,
      G1Arm5JointIndex::RIGHT_ELBOW_ROLL,
      G1Arm5JointIndex::WAIST_YAW,
      G1Arm5JointIndex::WAIST_ROLL,
      G1Arm5JointIndex::WAIST_PITCH};
#elif ARM_TYPE == G1ARM7
  static constexpr int NUM_ARM_JOINTS = 17;
  static constexpr auto NOT_USED_JOINT = G1Arm7JointIndex::NOT_USED_JOINT;
  std::array<G1Arm7JointIndex, NUM_ARM_JOINTS> arm_joints_ = {
      // Left arm
      G1Arm7JointIndex::LEFT_SHOULDER_PITCH,
      G1Arm7JointIndex::LEFT_SHOULDER_ROLL,
      G1Arm7JointIndex::LEFT_SHOULDER_YAW,
      G1Arm7JointIndex::LEFT_ELBOW,
      G1Arm7JointIndex::LEFT_WRIST_ROLL,
      G1Arm7JointIndex::LEFT_WRIST_PITCH,
      G1Arm7JointIndex::LEFT_WRIST_YAW,
      // Right arm
      G1Arm7JointIndex::RIGHT_SHOULDER_PITCH,
      G1Arm7JointIndex::RIGHT_SHOULDER_ROLL,
      G1Arm7JointIndex::RIGHT_SHOULDER_YAW,
      G1Arm7JointIndex::RIGHT_ELBOW,
      G1Arm7JointIndex::RIGHT_WRIST_ROLL,
      G1Arm7JointIndex::RIGHT_WRIST_PITCH,
      G1Arm7JointIndex::RIGHT_WRIST_YAW,
      // Waist
      G1Arm7JointIndex::WAIST_YAW,
      G1Arm7JointIndex::WAIST_ROLL,
      G1Arm7JointIndex::WAIST_PITCH,
  };
#endif
 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller") {
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 10,
        [this](std::shared_ptr<const unitree_hg::msg::LowState> msg) {
          StateCallback(msg);
        });

    using namespace std::chrono_literals;
    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001F));

    init_pos_.fill(0.F);
    target_pos_ = {0.F, PI_2, 0.F, PI_2, 0.F, 0.F, -PI_2,
                   0.F, PI_2, 0.F, 0.F,  0.F, 0.F};

    // start
    timer_ = std::thread([this] {
      while (true) {
        ControlLoop();
        std::this_thread::sleep_for(200ms);
      }
    });
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  std::thread timer_;
  LowState last_state_;
  bool state_received_ = false;

  std::array<float, NUM_ARM_JOINTS> init_pos_{}, target_pos_{};
  std::array<float, NUM_ARM_JOINTS> current_jpos_{}, current_jpos_des_{};

  float weight_{0.0F}, control_dt_{0.02F};
  float kp_ = 60.F, kd_ = 1.5F, dq_ = 0.F, tau_ff_ = 0.F;
  std::chrono::milliseconds sleep_time_{};

  void StateCallback(
      const std::shared_ptr<const unitree_hg::msg::LowState> msg) {
    last_state_ = *msg;
    state_received_ = true;
  }

  void ControlLoop() {
    if (!state_received_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for LowState...");
      return;
    }

    // initialize current joint positions
    for (uint64_t i = 0; i < arm_joints_.size(); ++i) {
      current_jpos_[i] =
          last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing arms...");
    MoveToPose(init_pos_, 2.0F);

    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Lifting arms...");
    MoveToPose(target_pos_, 5.0F);

    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Putting arms down...");
    MoveToPose(init_pos_, 5.0F);

    rclcpp::sleep_for(1s);
    StopControl();
  }

  void MoveToPose(const std::array<float, NUM_ARM_JOINTS> &target,
                  float duration) {
    int const steps = static_cast<int>(duration / control_dt_);
    float const max_delta = 0.5F * control_dt_;  // max joint velocity * dt

    for (int i = 0; i < steps; ++i) {
      for (int j = 0; j < static_cast<int>(arm_joints_.size()); ++j) {
        float const diff = target[j] - current_jpos_des_[j];
        current_jpos_des_[j] += std::clamp(diff, -max_delta, max_delta);
      }

      LowCmd cmd;
      for (int j = 0; j < static_cast<int>(arm_joints_.size()); ++j) {
        int const idx = static_cast<int>(arm_joints_[j]);
        cmd.motor_cmd[idx].q = current_jpos_des_[j];
        cmd.motor_cmd[idx].dq = dq_;
        cmd.motor_cmd[idx].kp = kp_;
        cmd.motor_cmd[idx].kd = kd_;
        cmd.motor_cmd[idx].tau = tau_ff_;
      }

      cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;

      pub_->publish(cmd);
      rclcpp::sleep_for(sleep_time_);
    }
  }

  void StopControl() {
    int const steps = static_cast<int>(2.0F / control_dt_);
    float const delta_w = 0.2F * control_dt_;

    for (int i = 0; i < steps; ++i) {
      weight_ -= delta_w;
      weight_ = std::clamp(weight_, 0.F, 1.F);

      LowCmd cmd;
      cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = weight_;
      pub_->publish(cmd);
      rclcpp::sleep_for(sleep_time_);
    }

    RCLCPP_INFO(this->get_logger(), "Arm control stopped.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmLowLevelController>());
  rclcpp::shutdown();
  return 0;
}
