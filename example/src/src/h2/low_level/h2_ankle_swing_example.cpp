/**
 * This example demonstrates how to use ROS2 to control ankle commands of
 * unitree h2 robot
 **/
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>

#include "common/motor_crc_hg.h"
#include "gamepad.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

constexpr float PI = 3.14159265358979323846F;

static constexpr const char *HG_CMD_TOPIC = "rt/lowcmd";
static constexpr const char *HG_IMU_TORSO = "rt/secondary_imu";
static constexpr const char *HG_STATE_TOPIC = "rt/lowstate";

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &new_data) {
    std::lock_guard<std::mutex> const lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() {
    std::lock_guard<std::mutex> const lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::mutex mutex_;
};

static constexpr int H2_NUM_MOTOR = 31;

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};

struct MotorCommand {
  std::array<float, H2_NUM_MOTOR> q_target = {};
  std::array<float, H2_NUM_MOTOR> dq_target = {};
  std::array<float, H2_NUM_MOTOR> kp = {};
  std::array<float, H2_NUM_MOTOR> kd = {};
  std::array<float, H2_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
  std::array<float, H2_NUM_MOTOR> q = {};
  std::array<float, H2_NUM_MOTOR> dq = {};
};

static const std::array<float, H2_NUM_MOTOR> Kp{
    150, 150, 150, 250, 60, 90,
    150, 150, 150, 250, 60, 90,
    200, 200, 200,
    90,  60,  20,  60,  4,  4,  4,
    90,  60,  20,  60,  4,  4,  4,
    30,  30,
};

static const std::array<float, H2_NUM_MOTOR> Kd{
    2.0, 2.0, 2.0, 2.0, 0.3, 0.1,
    2.0, 2.0, 2.0, 2.0, 0.3, 0.1,
    2.5, 5.0, 5.0,
    2.0, 1.0, 0.4, 1.0, 0.2, 0.2, 0.2,
    2.0, 1.0, 0.4, 1.0, 0.2, 0.2, 0.2,
    1.0, 1.0,
};

enum class Mode {
  PR = 0,
  AB = 1
};

enum H2JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnkleRoll = 4,
  LeftAnkleRollRaw = 4,
  LeftAnklePitch = 5,
  LeftAnklePitchRaw = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnkleRoll = 10,
  RightAnkleRollRaw = 10,
  RightAnklePitch = 11,
  RightAnklePitchRaw = 11,
  WaistYaw = 12,
  WaistRoll = 13,
  WaistA = 13,
  WaistPitch = 14,
  WaistB = 14,
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristpitch = 20,
  LeftWristyaw = 21,
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristpitch = 27,
  RightWristyaw = 28,
  HEAD_PITCH = 29,
  HEAD_YAW = 30,
};

class H2AnkleSwingSender : public rclcpp::Node {
 public:
  H2AnkleSwingSender() : Node("h2_ankle_swing_example"), mode_machine_(0) {
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        HG_STATE_TOPIC, 1,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    imustate_subscriber_ = this->create_subscription<unitree_hg::msg::IMUState>(
        HG_IMU_TORSO, 1,
        [this](const unitree_hg::msg::IMUState::SharedPtr message) {
          ImuStateHandler(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), [this] { Control(); });
    command_writer_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), [this] { LowCommandWriter(); });
  }

 private:
  void Control() {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        for (int i = 0; i < H2_NUM_MOTOR; ++i) {
          const double ratio = clamp(time_ / duration_, 0.0, 1.0);
          motor_command_tmp.q_target.at(i) =
              static_cast<float>((1.0 - ratio) * ms->q.at(i));
        }
      } else if (time_ < duration_ * 2) {
        mode_pr_ = Mode::PR;
        const double max_P = PI * 30.0 / 180.0;
        const double max_R = PI * 10.0 / 180.0;
        const double t = time_ - duration_;
        const double L_P_des = max_P * std::sin(2.0 * PI * t);
        const double L_R_des = max_R * std::sin(2.0 * PI * t);
        const double R_P_des = max_P * std::sin(2.0 * PI * t);
        const double R_R_des = -max_R * std::sin(2.0 * PI * t);

        motor_command_tmp.q_target.at(LeftAnklePitch) =
            static_cast<float>(L_P_des);
        motor_command_tmp.q_target.at(LeftAnkleRoll) =
            static_cast<float>(L_R_des);
        motor_command_tmp.q_target.at(RightAnklePitch) =
            static_cast<float>(R_P_des);
        motor_command_tmp.q_target.at(RightAnkleRoll) =
            static_cast<float>(R_R_des);
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }

  void LowCommandWriter() {
    unitree_hg::msg::LowCmd dds_low_command;
    dds_low_command.mode_pr = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc =
        motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < H2_NUM_MOTOR; ++i) {
        dds_low_command.motor_cmd.at(i).mode = 1;
        dds_low_command.motor_cmd.at(i).tau = mc->tau_ff.at(i);
        dds_low_command.motor_cmd.at(i).q = mc->q_target.at(i);
        dds_low_command.motor_cmd.at(i).dq = mc->dq_target.at(i);
        dds_low_command.motor_cmd.at(i).kp = mc->kp.at(i);
        dds_low_command.motor_cmd.at(i).kd = mc->kd.at(i);
      }

      get_crc(dds_low_command);
      lowcmd_publisher_->publish(dds_low_command);
    }
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr &message) {
    MotorState ms_tmp;
    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = message->motor_state[i].q;
      ms_tmp.dq.at(i) = message->motor_state[i].dq;
      if ((message->motor_state[i].motorstate != 0U) &&
          i <= RightAnkleRoll) {
        RCLCPP_INFO(this->get_logger(), "[ERROR] motor %d with code %u", i,
                    message->motor_state[i].motorstate);
      }
    }
    motor_state_buffer_.SetData(ms_tmp);

    ImuState imu_tmp;
    imu_tmp.omega = message->imu_state.gyroscope;
    imu_tmp.rpy = message->imu_state.rpy;
    imu_state_buffer_.SetData(imu_tmp);

    unitree::common::REMOTE_DATA_RX rx;
    std::memcpy(rx.buff, message->wireless_remote.data(), 40);
    gamepad_.update(rx.RF_RX);

    if (mode_machine_ != message->mode_machine) {
      if (mode_machine_ == 0) {
        RCLCPP_INFO(this->get_logger(), "H2 type: %u",
                    static_cast<unsigned>(message->mode_machine));
      }
      mode_machine_ = message->mode_machine;
    }

    if (++counter_ % 500 == 0) {
      counter_ = 0;
      const auto &rpy = message->imu_state.rpy;
      RCLCPP_INFO(this->get_logger(), "IMU.pelvis.rpy: %.2f %.2f %.2f",
                  rpy[0], rpy[1], rpy[2]);

      RCLCPP_INFO(this->get_logger(), "gamepad_.A.pressed: %d",
                  static_cast<int>(gamepad_.A.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad_.B.pressed: %d",
                  static_cast<int>(gamepad_.B.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad_.X.pressed: %d",
                  static_cast<int>(gamepad_.X.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad_.Y.pressed: %d",
                  static_cast<int>(gamepad_.Y.pressed));

      const auto &ms = message->motor_state;
      RCLCPP_INFO(this->get_logger(), "All %d Motors:", H2_NUM_MOTOR);

      std::ostringstream oss;
      oss << "mode: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << static_cast<unsigned>(ms[i].mode) << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "pos: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].q << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "vel: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].dq << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "tau_est: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].tau_est << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "temperature: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << ms[i].temperature[0] << "," << ms[i].temperature[1] << ";";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "vol: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].vol << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "sensor: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << ms[i].sensor[0] << "," << ms[i].sensor[1] << ";";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "motorstate: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << ms[i].motorstate << ",";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      oss.str("");
      oss << "reserve: ";
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << ms[i].reserve[0] << "," << ms[i].reserve[1] << ","
            << ms[i].reserve[2] << "," << ms[i].reserve[3] << ";";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
  }

  void ImuStateHandler(const unitree_hg::msg::IMUState::SharedPtr &message) {
    const auto &rpy = message->rpy;
    if (counter_ % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU.torso.rpy: %.2f %.2f %.2f", rpy[0],
                  rpy[1], rpy[2]);
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

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr imustate_subscriber_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr command_writer_timer_;

  unitree::common::Gamepad gamepad_;

  double time_{0.0};
  double control_dt_{0.002};
  double duration_{3.0};
  int32_t counter_{0};
  Mode mode_pr_{Mode::PR};
  uint8_t mode_machine_{0};

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<H2AnkleSwingSender>());
  rclcpp::shutdown();
  return 0;
}
