#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "motor_crc_hg.h"
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "g1/g1_motion_switch_client.hpp"
using namespace std::chrono_literals;

const int G1_NUM_MOTOR = 29;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &new_data) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::shared_mutex mutex_;
};

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};

struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

enum MotorType { GEARBOX_S = 0, GEARBOX_M = 1, GEARBOX_L = 2 };

const std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    // clang-format off
    // legs
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S,
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S,
    // waist
    GEARBOX_M, GEARBOX_S, GEARBOX_S,
    // arms
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S,
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S
    // clang-format on
};

enum PRorAB { PR = 0, AB = 1 };

enum G1JointValidIndex {
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_ROLL = 19,
  LEFT_WRIST_PITCH = 20,
  LEFT_WRIST_YAW = 21,
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_ROLL = 26,
  RIGHT_WRIST_PITCH = 27,
  RIGHT_WRIST_YAW = 28
};

inline uint32_t Crc32Core(const uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if ((CRC32 & 0x80000000) != 0U) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if ((data & xbit) != 0U) {
        CRC32 ^= dwPolynomial;
      }

      xbit >>= 1;
    }
  }
  return CRC32;
}

float GetMotorKp(MotorType type) {
  switch (type) {
    case GEARBOX_S:
    case GEARBOX_M:
      return 40;
    case GEARBOX_L:
      return 100;
    default:
      return 0;
  }
}

float GetMotorKd(MotorType type) {
  switch (type) {
    case GEARBOX_S:
    case GEARBOX_M:
    case GEARBOX_L:
      return 1;
    default:
      return 0;
  }
}

class G1Example : public rclcpp::Node {
 private:
  double time_{0.0};
  double control_dt_{0.002};  // [2ms]
  double duration_{3.0};      // [3 s]
  PRorAB mode_{PR};
  uint8_t mode_machine_{0};
  std::vector<std::vector<double>> frames_data_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      lowstate_subscriber_;
  rclcpp::TimerBase::SharedPtr command_writer_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::string resource_dir_;
  std::shared_ptr<unitree::robot::g1::MotionSwitchClient> client_;
  std::thread thread_;

 public:
  explicit G1Example(std::string resource_dir)
      : Node("g1_example"), resource_dir_(std::move(resource_dir)) {
    client_ = std::make_shared<unitree::robot::g1::MotionSwitchClient>(this);

    thread_ = std::thread([this]() {
      std::this_thread::sleep_for(1s);
      while (queryMotionStatus() != 0) {
        std::cout << "Try to deactivate the motion control-related service."
                  << std::endl;
        int32_t ret = client_->ReleaseMode();
        if (ret == 0) {
          std::cout << "ReleaseMode succeeded." << std::endl;
        } else {
          std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        std::this_thread::sleep_for(2s);
      }

      // Initialize publishers and subscribers
      lowcmd_publisher_ =
          this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);
      lowstate_subscriber_ =
          this->create_subscription<unitree_hg::msg::LowState>(
              "lowstate", 10,
              [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                LowStateHandler(msg);
              });

      // Initialize timers
      command_writer_timer_ =
          this->create_wall_timer(2ms, [this] { LowCommandWriter(); });
      control_timer_ = this->create_wall_timer(2ms, [this] { Control(); });

      RCLCPP_INFO(this->get_logger(), "G1 Example Node Initialized");
    });
  }

  int queryMotionStatus() {
    std::string robotForm;
    std::string motionName;
    int motionStatus = 0;
    int32_t ret = client_->CheckMode(robotForm, motionName);
    if (ret == 0) {
      std::cout << "CheckMode succeeded." << std::endl;
    } else {
      std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty()) {
      std::cout << "The motion control-related service is deactivated."
                << std::endl;
      motionStatus = 0;
    } else {
      std::string serviceName = queryServiceName(robotForm, motionName);
      std::cout << "Service: " << serviceName << " is activate" << std::endl;
      motionStatus = 1;
    }
    return motionStatus;
  }

  static std::string queryServiceName(const std::string &form,
                                      const std::string &name) {
    if (form == "0") {
      if (name == "normal") {
        return "sport_mode";
      }
      if (name == "ai") {
        return "ai_sport";
      }
      if (name == "advanced") {
        return "advanced_sport";
      }
    } else {
      if (name == "ai-w") {
        return "wheeled_sport(go2W)";
      }
      if (name == "normal-w") {
        return "wheeled_sport(b2W)";
      }
    }
    return "";
  }

  void loadBehaviorLibrary(const std::string &behavior_name) {
    YAML::Node motion = YAML::LoadFile(resource_dir_ + behavior_name + ".seq");

    auto content = motion["components"][1]["content"].as<std::string>();
    int num_parts = motion["components"][1]["num_parts"].as<int>();
    RCLCPP_INFO(this->get_logger(), "BehaviorName: %s.seq",
                behavior_name.c_str());
    RCLCPP_INFO(this->get_logger(), "%s with %d", content.c_str(), num_parts);

    auto frames = motion["components"][1]["frames"];

    for (const auto &frame : frames) {
      std::vector<double> frame_data;
      for (const auto &element : frame) {
        frame_data.push_back(element.as<double>());
      }
      frames_data_.push_back(frame_data);
    }

    RCLCPP_INFO(this->get_logger(), "%zu knots with %zu DOF",
                frames_data_.size(), frames_data_[0].size());
  }

  void ReportRPY() {
    const std::shared_ptr<const ImuState> imu_tmp_ptr =
        imu_state_buffer_.GetData();
    if (imu_tmp_ptr) {
      RCLCPP_INFO(this->get_logger(), "rpy: [%f, %f, %f]",
                  imu_tmp_ptr->rpy.at(0), imu_tmp_ptr->rpy.at(1),
                  imu_tmp_ptr->rpy.at(2));
    }
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr msg) {
    // if (msg->crc != Crc32Core((uint32_t *)msg.get(),  // NOLINT
    //                           (sizeof(unitree_hg::msg::LowState) >> 2) - 1))
    //                           {
    //   RCLCPP_ERROR(this->get_logger(), "low_state CRC Error");
    //   return;
    // }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = msg->motor_state[i].q;
      ms_tmp.dq.at(i) = msg->motor_state[i].dq;
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = {msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1],
                     msg->imu_state.gyroscope[2]};
    imu_tmp.rpy = {msg->imu_state.rpy[0], msg->imu_state.rpy[1],
                   msg->imu_state.rpy[2]};
    imu_state_buffer_.SetData(imu_tmp);

    // update mode machine
    if (mode_machine_ != msg->mode_machine) {
      if (mode_machine_ == 0) {
        RCLCPP_INFO(this->get_logger(), "G1 type: %u", msg->mode_machine);
      }
      mode_machine_ = msg->mode_machine;
    }
  }

  void LowCommandWriter() {
    auto dds_low_command = unitree_hg::msg::LowCmd();
    dds_low_command.mode_pr = mode_;
    dds_low_command.mode_machine = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc =
        motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd[i].tau = mc->tau_ff.at(i);
        dds_low_command.motor_cmd[i].q = mc->q_target.at(i);
        dds_low_command.motor_cmd[i].dq = mc->dq_target.at(i);
        dds_low_command.motor_cmd[i].kp = mc->kp.at(i);
        dds_low_command.motor_cmd[i].kd = mc->kd.at(i);
      }
 
      get_crc(dds_low_command);
      lowcmd_publisher_->publish(dds_low_command);
    }
  }

  void Control() {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);

          double q_des = 0;
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.q_target.at(i) =
              static_cast<float>((q_des - ms->q.at(i)) * ratio + ms->q.at(i));
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }
      } else {
        // [Stage 2]: tracking the offline trajectory
        auto frame_index =
            static_cast<size_t>((time_ - duration_) / control_dt_);
        if (frame_index >= frames_data_.size()) {
          frame_index = frames_data_.size() - 1;
          time_ = 0.0;  // RESET
        }

        if (frame_index % 100 == 0) {
          RCLCPP_INFO(this->get_logger(), "Frame Index: %zu", frame_index);
        }

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          size_t const index_in_frame = i - LEFT_SHOULDER_PITCH;
          auto value =
              static_cast<float>(frames_data_[frame_index][index_in_frame]);
          motor_command_tmp.q_target.at(i) =
              (i >= LEFT_SHOULDER_PITCH) ? value : 0.0F;
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
                 "Usage: %s <resource_directory> [behavior_name], for example: g1_dual_arm_example ./behavior_lib/ motion", argv[0]);
    return 1;
  }

  std::string resource_dir = argv[1];
  auto node = std::make_shared<G1Example>(resource_dir);

  // Optional: Load behavior if specified
  if (argc > 2) {
    std::string behavior_name = argv[2];
    node->loadBehaviorLibrary(behavior_name);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}