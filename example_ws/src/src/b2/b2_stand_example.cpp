#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "b2/b2_motion_switch_client.hpp"
#include "motor_crc.h"

#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_LOWSTATE "/lowstate"

class Custom : public rclcpp::Node {
 public:
  explicit Custom() : Node("low_level_cmd_node"), msc_(this) {
    Init();
    Start();
  }

  void Init();
  void Start();

 private:
  void InitLowCmd();
  void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
  void LowCmdWrite();
  int queryMotionStatus();
  std::string queryServiceName(std::string form, std::string name);

  float kp_ = 1000.0;
  float kd_ = 10.0;
  double time_consume_ = 0;
  int rate_count_ = 0;
  int sin_count_ = 0;
  int motiontime_ = 0;
  float dt_ = 0.002;  // 0.001~0.01

  unitree::robot::b2::MotionSwitchClient msc_;
  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber;
  rclcpp::TimerBase::SharedPtr lowCmdWriteThreadPtr;

  float target_pos_1_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  float target_pos_2_[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  float target_pos_3_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              0.0, 1.36, -2.65, 0.0, 1.36, -2.65};

  float target_pos_4_[12] = {-0.5, 1.36, -2.65, 0.5, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  float start_pos_[12]{};
  float duration_1_ = 500;
  float duration_2_ = 900;
  float duration_3_ = 1000;
  float duration_4_ = 1100;
  float duration_5_ = 500;
  float percent_1_ = 0;
  float percent_2_ = 0;
  float percent_3_ = 0;
  float percent_4_ = 0;
  float percent_5_ = 0;

  bool first_run_ = true;
  bool done_ = false;
};

void Custom::Init() {
  InitLowCmd();

  lowcmd_publisher = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  lowstate_subscriber = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
  
  while(queryMotionStatus()) {
      std::cout << "Try to deactivate the motion control-related service." << std::endl;
      int32_t ret = msc_.ReleaseMode(); 
      if (ret == 0) {
          std::cout << "ReleaseMode succeeded." << std::endl;
      } else {
          std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
      }
     std::this_thread::sleep_for(std::chrono::seconds(5));
  }
}

void Custom::InitLowCmd() {
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd[i].mode = (0x0A);  // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd[i].q = (PosStopF);
    low_cmd_.motor_cmd[i].kp = (0);
    low_cmd_.motor_cmd[i].dq = (VelStopF);
    low_cmd_.motor_cmd[i].kd = (0);
    low_cmd_.motor_cmd[i].tau = (0);
  }
}

int Custom::queryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = msc_.CheckMode(robotForm, motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }

    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = queryServiceName(robotForm, motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string Custom::queryServiceName(std::string form, std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode"; 
        if(name == "ai" ) return "ai_sport"; 
        if(name == "advanced" ) return "advanced_sport"; 
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

void Custom::Start() {
  /*loop publishing thread*/
  lowCmdWriteThreadPtr = this->create_wall_timer(std::chrono::milliseconds(2), [this] {
    LowCmdWrite();
  });
}

void Custom::LowStateMessageHandler(
    const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
}

void Custom::LowCmdWrite() {
  if (percent_5_ < 1) {
    std::cout << "Read sensor data example: " << std::endl;
    std::cout << "Joint 0 pos: " << low_state_.motor_state[0].q << std::endl;
    std::cout << "Imu accelerometer : " << "x: "
              << low_state_.imu_state.accelerometer[0]
              << " y: " << low_state_.imu_state.accelerometer[1]
              << " z: " << low_state_.imu_state.accelerometer[2] << std::endl;
    std::cout << "Foot force " << low_state_.foot_force[0] << std::endl;
    std::cout << std::endl;
  }
  if ((percent_5_ == 1) && (!done_)) {
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
    
    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ == 1) && (percent_4_ < 1)) {
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

    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ == 1) && (percent_4_ == 1) && ((percent_5_ <= 1))) {
      percent_5_ += static_cast<float>(1) / duration_5_;
      percent_5_ = percent_5_ > 1 ? 1 : percent_5_;
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_5_) * target_pos_3_[j] + percent_5_ * target_pos_4_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    
    get_crc(low_cmd_); 
    
    lowcmd_publisher->publish(low_cmd_);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Custom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}