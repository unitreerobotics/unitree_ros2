/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 *unitree h1_2 robot
 **/
#include "common/motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

constexpr bool INFO_IMU = false;    // Set 1 to info IMU states
constexpr bool INFO_MOTOR = false;  // Set 1 to info motor states
constexpr bool HIGH_FREQ = true;
// Set 1 to subscribe to low states with high frequencies (500Hz)

const int H1_2_NUM_MOTOR = 27;

enum PRorAB { PR = 0, AB = 1 };

enum H12JointIndex {
  // legs
  LEFT_HIP_YAW = 0,
  LEFT_HIP_PITCH = 1,
  LEFT_HIP_ROLL = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_B = 4,
  LEFT_ANKLE_ROLL = 5,
  LEFT_ANKLE_A = 5,
  RIGHT_HIP_YAW = 6,
  RIGHT_HIP_PITCH = 7,
  RIGHT_HIP_ROLL = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_B = 10,
  RIGHT_ANKLE_ROLL = 11,
  RIGHT_ANKLE_A = 11,
  // torso
  WAIST_YAW = 12,
  // arms
  LEFT_SHOULDER_PITCH = 13,
  LEFT_SHOULDER_ROLL = 14,
  LEFT_SHOULDER_YAW = 15,
  LEFT_ELBOW = 16,
  LEFT_WRIST_ROLL = 17,
  LEFT_WRIST_PITCH = 18,
  LEFT_WRIST_YAW = 19,
  RIGHT_SHOULDER_PITCH = 20,
  RIGHT_SHOULDER_ROLL = 21,
  RIGHT_SHOULDER_YAW = 22,
  RIGHT_ELBOW = 23,
  RIGHT_WRIST_ROLL = 24,
  RIGHT_WRIST_PITCH = 25,
  RIGHT_WRIST_YAW = 26
};

// Create a low_level_cmd_sender class for low state receive
class LowLevelCmdSender : public rclcpp::Node {
 public:
  LowLevelCmdSender() : Node("low_level_cmd_sender") {
    const auto *topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }

    // The suber  callback function is bind to
    // low_level_cmd_sender::topic_callback
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    // the lowcmd_publisher_ is set to subscribe "/lowcmd" topic
    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

    // The timer is set to 500 Hz, and bind to low_level_cmd_sender::Control
    // function
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dt_),
                                     [this] { Control(); });

    // Running time count
    time_ = 0;

    duration_ = 3;  // 3 s
  }

 private:
  void Control() {
    // Test code here
    time_ += control_dt_;
    low_command_.mode_pr = mode_;
    low_command_.mode_machine = mode_machine_;
    for (int i = 0; i < H1_2_NUM_MOTOR; ++i) {
      low_command_.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
      low_command_.motor_cmd[i].tau = 0.0;
      low_command_.motor_cmd[i].q = 0.0;
      low_command_.motor_cmd[i].dq = 0.0;
      low_command_.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
      low_command_.motor_cmd[i].kd = 1.0;
    }

    if (time_ < duration_) {
      // [Stage 1]: set robot to zero posture
      for (int i = 0; i < H1_2_NUM_MOTOR; ++i) {
        double const ratio = clamp(time_ / duration_, 0.0, 1.0);
        low_command_.motor_cmd[i].q = (1. - ratio) * motor_[i].q;
      }
    } else {
      // [Stage 2]: swing ankle's PR
      mode_ = PRorAB::PR;  // Enable PR mode
      // generate sin/cos trajectory
      double const max_P = 0.25;  // [rad]
      double const max_R = 0.25;  // [rad]
      double const t = time_ - duration_;
      double const L_P_des = max_P * std::cos(2.0 * M_PI * t);
      double const L_R_des = max_R * std::sin(2.0 * M_PI * t);
      double const R_P_des = max_P * std::cos(2.0 * M_PI * t);
      double const R_R_des = -max_R * std::sin(2.0 * M_PI * t);

      // update ankle joint position targets
      float const Kp_Pitch = 80;
      float const Kd_Pitch = 1;
      float const Kp_Roll = 80;
      float const Kd_Roll = 1;

      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_PITCH].q = L_P_des;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_PITCH].dq = 0;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_PITCH].kp = Kp_Pitch;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_PITCH].kd = Kd_Pitch;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_PITCH].tau = 0;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_ROLL].q = L_R_des;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_ROLL].dq = 0;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_ROLL].kp = Kp_Roll;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_ROLL].kd = Kd_Roll;
      low_command_.motor_cmd[H12JointIndex::LEFT_ANKLE_ROLL].tau = 0;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_PITCH].q = R_P_des;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_PITCH].dq = 0;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_PITCH].kp = Kp_Pitch;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_PITCH].kd = Kd_Pitch;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_PITCH].tau = 0;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_ROLL].q = R_R_des;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_ROLL].dq = 0;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_ROLL].kp = Kp_Roll;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_ROLL].kd = Kd_Roll;
      low_command_.motor_cmd[H12JointIndex::RIGHT_ANKLE_ROLL].tau = 0;

      double const max_wrist_roll_angle = 0.5;  // [rad]
      double const WristRoll_des =
          max_wrist_roll_angle * std::sin(2.0 * M_PI * t);
      low_command_.motor_cmd[H12JointIndex::LEFT_WRIST_ROLL].q = WristRoll_des;
      low_command_.motor_cmd[H12JointIndex::LEFT_WRIST_ROLL].dq = 0;
      low_command_.motor_cmd[H12JointIndex::LEFT_WRIST_ROLL].kp = 50;
      low_command_.motor_cmd[H12JointIndex::LEFT_WRIST_ROLL].kd = 1;
      low_command_.motor_cmd[H12JointIndex::LEFT_WRIST_ROLL].tau = 0;

      low_command_.motor_cmd[H12JointIndex::RIGHT_WRIST_ROLL].q = WristRoll_des;
      low_command_.motor_cmd[H12JointIndex::RIGHT_WRIST_ROLL].dq = 0;
      low_command_.motor_cmd[H12JointIndex::RIGHT_WRIST_ROLL].kp = 50;
      low_command_.motor_cmd[H12JointIndex::RIGHT_WRIST_ROLL].kd = 1;
      low_command_.motor_cmd[H12JointIndex::RIGHT_WRIST_ROLL].tau = 0;
    }
    get_crc(low_command_);
    lowcmd_publisher_->publish(low_command_);  // Publish lowcmd message
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr &message) {
    mode_machine_ = static_cast<int>(message->mode_machine);
    imu_ = message->imu_state;
    for (int i = 0; i < H1_2_NUM_MOTOR; i++) {
      motor_[i] = message->motor_state[i];
    }

    if (INFO_IMU) {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      RCLCPP_INFO(this->get_logger(),
                  "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                  imu_.rpy[1], imu_.rpy[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                  imu_.quaternion[3]);
      RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
      RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu_.accelerometer[0], imu_.accelerometer[1],
                  imu_.accelerometer[2]);
    }
    if (INFO_MOTOR) {
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque
      for (int i = 0; i < H1_2_NUM_MOTOR; i++) {
        motor_[i] = message->motor_state[i];
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                    motor_[i].tau_est);
      }
    }
  }

  static double clamp(double value, double low, double high) {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  }

  rclcpp::TimerBase::SharedPtr timer_;  // ROS2 timer
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr
      lowcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      lowstate_subscriber_;              // ROS2 Subscriber
  unitree_hg::msg::LowCmd low_command_;  // Unitree hg lowcmd message
  unitree_hg::msg::IMUState imu_;        // Unitree hg IMU message
  unitree_hg::msg::MotorState
      motor_[H1_2_NUM_MOTOR];  // Unitree hg motor state message
  double control_dt_ = 0.002;  // 2ms
  int timer_dt_ = control_dt_ * 1000;
  double time_;  // Running time count
  double duration_;
  PRorAB mode_ = PRorAB::PR;
  int mode_machine_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::TimerBase::SharedPtr const
      timer_;  // Create a timer callback object to send cmd in time intervals
  auto node =
      std::make_shared<LowLevelCmdSender>();  // Create a ROS2 node and make
                                              // share with
                                              // low_level_cmd_sender class
  rclcpp::spin(node);                         // Run ROS2 node
  rclcpp::shutdown();                         // Exit
  return 0;
}
