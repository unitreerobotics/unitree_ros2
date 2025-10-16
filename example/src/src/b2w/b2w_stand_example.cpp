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

  float Kp = 1000.0;
  float Kd = 10.0;
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt_ = 0.002;  // 0.001~0.01

  unitree::robot::b2::MotionSwitchClient msc_;
  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber;
  rclcpp::TimerBase::SharedPtr lowCmdWriteThreadPtr;

  float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                            -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  float _targetPos_3[12] = {-0.65, 1.36, -2.65, 0.65, 1.36, -2.65,
                            -0.65, 1.36, -2.65, 0.65, 1.36, -2.65};

  float _startPos[12];
  float _duration_1 = 800;
  float _duration_2 = 800;
  float _duration_3 = 2000;
  float _duration_4 = 1500;
  float _percent_1 = 0;
  float _percent_2 = 0;
  float _percent_3 = 0;
  float _percent_4 = 0;

  bool firstRun = true;
  bool done = false;
};

void Custom::Init() {
  InitLowCmd();

  lowcmd_publisher = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  lowstate_subscriber = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
}

void Custom::InitLowCmd() {
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd[i].mode = (0x01);
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
  if (_percent_4 < 1) {
    std::cout << "Read sensor data example: " << std::endl;
    std::cout << "Joint 0 pos: " << low_state_.motor_state[0].q << std::endl;
    std::cout << "Imu accelerometer : " << "x: "
              << low_state_.imu_state.accelerometer[0]
              << " y: " << low_state_.imu_state.accelerometer[1]
              << " z: " << low_state_.imu_state.accelerometer[2] << std::endl;
    std::cout << "Foot force " << low_state_.foot_force[0] << std::endl;
    std::cout << std::endl;
  }
  if ((_percent_4 == 1) && (!done)) {
    std::cout << "The example is done! " << std::endl;
    std::cout << std::endl;
    done = true;
  }

  motiontime++;
  if (motiontime >= 500) {
    if (firstRun) {
      for (int i = 0; i < 12; i++) {
        _startPos[i] = low_state_.motor_state[i].q;
      }
      firstRun = false;
    }

    _percent_1 += static_cast<float>(1) / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
    if (_percent_1 < 1) {
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = Kp;
        low_cmd_.motor_cmd[j].kd = Kd;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    if ((_percent_1 == 1) && (_percent_2 < 1)) {
      _percent_2 += static_cast<float>(1) / _duration_2;
      _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = Kp;
        low_cmd_.motor_cmd[j].kd = Kd;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }

    if ((_percent_1 == 1) && (_percent_2 == 1) && (_percent_3 < 1)) {
      _percent_3 += static_cast<float>(1) / _duration_3;
      _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q = _targetPos_2[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = Kp;
        low_cmd_.motor_cmd[j].kd = Kd;
        low_cmd_.motor_cmd[j].tau = 0;
      }
      
      if (_percent_3 < 0.4) {
        for (int j = 12; j < 16; j++) {
          low_cmd_.motor_cmd[j].q = 0;
          low_cmd_.motor_cmd[j].kp = 0;
          low_cmd_.motor_cmd[j].dq = 3;
          low_cmd_.motor_cmd[j].kd = Kd;
          low_cmd_.motor_cmd[j].tau = 0;
        }
      } else if ((_percent_3 >= 0.4) && (_percent_3 < 0.8)) {
        for (int j = 12; j < 16; j++) {
          low_cmd_.motor_cmd[j].q = 0;
          low_cmd_.motor_cmd[j].kp = 0;
          low_cmd_.motor_cmd[j].dq = -3;
          low_cmd_.motor_cmd[j].kd = Kd;
          low_cmd_.motor_cmd[j].tau = 0;
        }
      } else if (_percent_3 >= 0.8) {
        for (int j = 12; j < 16; j++) {
          low_cmd_.motor_cmd[j].q = 0;
          low_cmd_.motor_cmd[j].kp = 0;
          low_cmd_.motor_cmd[j].dq = 0;
          low_cmd_.motor_cmd[j].kd = Kd;
          low_cmd_.motor_cmd[j].tau = 0;
        }
      }
    }
    
    if ((_percent_1 == 1) && (_percent_2 == 1) && (_percent_3 == 1) &&
        ((_percent_4 <= 1))) {
      _percent_4 += static_cast<float>(1) / _duration_4;
      _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = Kp;
        low_cmd_.motor_cmd[j].kd = Kd;
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