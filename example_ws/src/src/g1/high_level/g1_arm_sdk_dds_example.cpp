#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
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
  std::array<float, NUM_ARM_JOINTS> target_pos_ = {
      0.0F, PI_2,  0.0F, PI_2, 0.0F,  // left
      0.0F, -PI_2, 0.0F, PI_2, 0.0F,  // right
      0.0F, 0.0F,  0.0F};
#elif ARM_TYPE == G1ARM7
  static constexpr int NUM_ARM_JOINTS = 17;
  static constexpr auto NOT_USED_JOINT = G1Arm7JointIndex::NOT_USED_JOINT;
  std::array<G1Arm7JointIndex, NUM_ARM_JOINTS> arm_joints_ = {
      G1Arm7JointIndex::LEFT_SHOULDER_PITCH,
      G1Arm7JointIndex::LEFT_SHOULDER_ROLL,
      G1Arm7JointIndex::LEFT_SHOULDER_YAW,
      G1Arm7JointIndex::LEFT_ELBOW,
      G1Arm7JointIndex::LEFT_WRIST_ROLL,
      G1Arm7JointIndex::LEFT_WRIST_PITCH,
      G1Arm7JointIndex::LEFT_WRIST_YAW,
      G1Arm7JointIndex::RIGHT_SHOULDER_PITCH,
      G1Arm7JointIndex::RIGHT_SHOULDER_ROLL,
      G1Arm7JointIndex::RIGHT_SHOULDER_YAW,
      G1Arm7JointIndex::RIGHT_ELBOW,
      G1Arm7JointIndex::RIGHT_WRIST_ROLL,
      G1Arm7JointIndex::RIGHT_WRIST_PITCH,
      G1Arm7JointIndex::RIGHT_WRIST_YAW,
      G1Arm7JointIndex::WAIST_YAW,
      G1Arm7JointIndex::WAIST_ROLL,
      G1Arm7JointIndex::WAIST_PITCH};
  std::array<float, NUM_ARM_JOINTS> target_pos_ = {
      0.0F, PI_2,  0.0F, PI_2, 0.0F, 0.0F, 0.0F,  // left
      0.0F, -PI_2, 0.0F, PI_2, 0.0F, 0.0F, 0.0F,  // right
      0.0F, 0.F,   0.F};

#endif

 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller") {
    // ROS2接口初始化
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    sub_ = this->create_subscription<LowState>(
        "/lowstate", 10,
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    // 位置初始化
    init_pos_.fill(0.0F);

    thread_ = std::thread([this]() { ControlLoop(); });
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  std::thread thread_;

  LowState last_state_;
  std::mutex state_mutex_;
  bool state_received_ = false;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};
  std::chrono::milliseconds sleep_time_{};

  std::array<float, NUM_ARM_JOINTS> init_pos_{};

  std::array<float, NUM_ARM_JOINTS> current_jpos_{};

  void StateCallback(const LowState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_ = *msg;

    if (state_received_) {
      return;
    }
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      current_jpos_[i] =
          last_state_.motor_state[static_cast<int>(arm_joints_[i])].q;
    }

    state_received_ = true;
  }

  void ControlLoop() {
    while (!state_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waiting for LowState...");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "LowState received. Starting control...");
    StartControlSequence();
  }

  // start control sequence
  void StartControlSequence() {
    RCLCPP_INFO(this->get_logger(), "Starting control sequence...");
    auto start_pos = current_jpos_;
    // first stage: move to initial position
    RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
    MoveTo(init_pos_, current_jpos_, 3.0F, true);

    // second stage: lift arms
    RCLCPP_INFO(this->get_logger(), "Lifting arms...");
    MoveTo(target_pos_, current_jpos_, 5.0F, false);

    // third stage: put arms down
    RCLCPP_INFO(this->get_logger(), "Putting arms down...");
    MoveTo(init_pos_, current_jpos_, 5.0F, false);

    // third stage: put arms down
    RCLCPP_INFO(this->get_logger(), "Putting arms down...");
    MoveTo(start_pos, current_jpos_, 3.0F, false);

    // final stage: stop control
    StopControl();
  }

  // move to target position from current position
  void MoveTo(const std::array<float, NUM_ARM_JOINTS>& target,
              std::array<float, NUM_ARM_JOINTS>& current, float duration,
              bool smooth) {
    const int steps = static_cast<int>(duration / control_dt_);
    const float max_delta = max_joint_velocity_ * control_dt_;

    for (int i = 0; i < steps; ++i) {
      float phase = static_cast<float>(i) / static_cast<float>(steps);

      for (size_t j = 0; j < arm_joints_.size(); ++j) {
        if (smooth) {
          // smooth mode: linear interpolation
          current[j] = current[j] * (1 - phase) + target[j] * phase;
        } else {
          // non-smooth mode: move with max velocity
          float diff = target[j] - current[j];
          current[j] += std::clamp(diff, -max_delta, max_delta);
        }
      }

      SendPositionCommand(current);
      std::this_thread::sleep_for(sleep_time_);
    }
  }

  void SendPositionCommand(const std::array<float, NUM_ARM_JOINTS>& positions) {
    LowCmd cmd;

    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      int idx = static_cast<int>(arm_joints_[i]);
      cmd.motor_cmd[idx].q = positions[i];
      cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      if (i >= arm_joints_.size() - 3) {
        cmd.motor_cmd[idx].kp = kp_ * 4.0F;
        cmd.motor_cmd[idx].kd = kd_ * 4.0F;
      } else {
        cmd.motor_cmd[idx].kp = kp_;
        cmd.motor_cmd[idx].kd = kd_;
      }
    }

    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 1.0F;

    pub_->publish(cmd);
  }

  void StopControl() {
    RCLCPP_INFO(this->get_logger(), "Stopping control...");

    const int steps = static_cast<int>(2.0F / control_dt_);
    const float delta_w = 0.2F * control_dt_;
    float weight = 1.0F;

    for (int i = 0; i < steps; ++i) {
      weight -= delta_w;
      weight = std::clamp(weight, 0.0F, 1.0F);

      LowCmd cmd;

      for (size_t j = 0; j < arm_joints_.size(); ++j) {
        int idx = static_cast<int>(arm_joints_[j]);
        cmd.motor_cmd[idx].q = current_jpos_[j];
        cmd.motor_cmd[idx].dq = 0.0F;
        cmd.motor_cmd[idx].kp = kp_;
        cmd.motor_cmd[idx].kd = kd_;
        cmd.motor_cmd[idx].tau = 0.0F;
      }
      cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = weight;
      pub_->publish(cmd);

      rclcpp::sleep_for(sleep_time_);
    }

    LowCmd cmd;
    cmd.motor_cmd[static_cast<int>(NOT_USED_JOINT)].q = 0.0F;
    pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Control stopped.");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmLowLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}