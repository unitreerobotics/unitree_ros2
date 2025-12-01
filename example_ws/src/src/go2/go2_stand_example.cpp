#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_LOWSTATE "/lowstate"

class LowLevelCmdNode : public rclcpp::Node {
 public:
  explicit LowLevelCmdNode() : Node("low_level_cmd_node") {
    Init();
    Start();
  }

  void Init();
  void Start();

 private:
  void InitLowCmd();
  void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
  void LowCmdWrite();
  std::string queryServiceName(std::string form, std::string name);

  float kp_ = 60.0;
  float kd_ = 5.0;
  double time_consume_ = 0;
  int rate_count_ = 0;
  int sin_count_ = 0;
  int motiontime_ = 0;
  float dt_ = 0.002;  // 0.001~0.01

  unitree_go::msg::LowCmd low_cmd_;      // default init
  unitree_go::msg::LowState low_state_;  // default init

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float target_pos_1_[12] = {0.0,  1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  float target_pos_2_[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  float target_pos_3_[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5,  1.36, -2.65, 0.5,  1.36, -2.65};

  float start_pos_[12]{};
  float duration_1_ = 500;
  float duration_2_ = 500;
  float duration_3_ = 1000;
  float duration_4_ = 900;
  float percent_1_ = 0;
  float percent_2_ = 0;
  float percent_3_ = 0;
  float percent_4_ = 0;

  bool first_run_ = true;
  bool done_ = false;
};

void LowLevelCmdNode::Init() {
  InitLowCmd();

  low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
  /*Shut down motion control-related service*/
}

void LowLevelCmdNode::InitLowCmd() {
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd[i].mode = (0x01);  // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd[i].q = (PosStopF);
    low_cmd_.motor_cmd[i].kp = (0);
    low_cmd_.motor_cmd[i].dq = (VelStopF);
    low_cmd_.motor_cmd[i].kd = (0);
    low_cmd_.motor_cmd[i].tau = (0);
  }
}

void LowLevelCmdNode::Start() {
  /*loop publishing thread*/
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] {
    LowCmdWrite();
    // timer_->cancel();
  });
}

void LowLevelCmdNode::LowStateMessageHandler(
    const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
}

void LowLevelCmdNode::LowCmdWrite() {
  if (percent_4_ < 1) {
    std::cout << "Read sensor data example: " << std::endl;
    std::cout << "Joint 0 pos: " << low_state_.motor_state[0].q << std::endl;
    std::cout << "Imu accelerometer : " << "x: "
              << low_state_.imu_state.accelerometer[0]
              << " y: " << low_state_.imu_state.accelerometer[1]
              << " z: " << low_state_.imu_state.accelerometer[2] << std::endl;
    std::cout << "Foot force " << low_state_.foot_force[0] << std::endl;
    std::cout << std::endl;
  }
  if ((percent_4_ == 1) && (!done_)) {
    std::cout << "The example is done! " << std::endl;
    std::cout << std::endl;
    done_ = true;
  }

  motiontime_++;
  if (motiontime_ >= 500) {
    if (first_run_) {
      for (int i = 0; i < 12; i++) {
        start_pos_[i] = low_state_.motor_state[i].q;
      }
      first_run_ = false;
    }

    percent_1_ += static_cast<float>(1) / duration_1_;
    percent_1_ = percent_1_ > 1 ? 1 : percent_1_;
    if (percent_1_ < 1) {
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_1_) * start_pos_[j] + percent_1_ * target_pos_1_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    if ((percent_1_ == 1) && (percent_2_ < 1)) {
      percent_2_ += static_cast<float>(1) / duration_2_;
      percent_2_ = percent_2_ > 1 ? 1 : percent_2_;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_2_) * target_pos_1_[j] + percent_2_ * target_pos_2_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }

    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ < 1)) {
      percent_3_ += static_cast<float>(1) / duration_3_;
      percent_3_ = percent_3_ > 1 ? 1 : percent_3_;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q = target_pos_2_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ == 1) &&
        ((percent_4_ <= 1))) {
      percent_4_ += static_cast<float>(1) / duration_4_;
      percent_4_ = percent_4_ > 1 ? 1 : percent_4_;
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_4_) * target_pos_2_[j] + percent_4_ * target_pos_3_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    get_crc(low_cmd_);  // Check motor cmd crc
    low_cmd_pub_->publish(low_cmd_);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LowLevelCmdNode>();
  node->Init();
  node->Start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
