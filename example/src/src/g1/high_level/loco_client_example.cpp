
#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <map>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"

class UnitreeG1ControlNode : public rclcpp::Node {
 public:
  explicit UnitreeG1ControlNode(const std::map<std::string, std::string> &args)
      : Node("unitree_g1_control_node"), args_(args), client_(this) {
    // Process commands
    thread_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ProcessCommands();
    });
  }
  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Execute action failed, error code: %d",
                 error_code);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID);
    UT_PRINT_ERR(error_code,
                 unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID);
    UT_PRINT_ERR(error_code, UT_ROBOT_TASK_TIMEOUT);
    return false;
  }

 private:
  std::thread thread_;
  void ProcessCommands() {  // NOLINT
    for (const auto &arg_pair : args_) {
      RCLCPP_INFO(this->get_logger(),
                  "Processing command: [%s] with param: [%s]...",
                  arg_pair.first.c_str(), arg_pair.second.c_str());

      if (arg_pair.first == "get_fsm_id") {
        int fsm_id = 0;
        auto ret = client_.GetFsmId(fsm_id);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetFsmId failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "current fsm_id: %d", fsm_id);
      }

      if (arg_pair.first == "get_fsm_mode") {
        int fsm_mode = 0;
        auto ret = client_.GetFsmMode(fsm_mode);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetFsmMode failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "current fsm_mode: %d", fsm_mode);
      }

      if (arg_pair.first == "get_balance_mode") {
        int balance_mode = 0;
        auto ret = client_.GetBalanceMode(balance_mode);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetBalanceMode failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "current balance_mode: %d",
                    balance_mode);
      }

      if (arg_pair.first == "get_swing_height") {
        float swing_height = NAN;
        auto ret = client_.GetSwingHeight(swing_height);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetSwingHeight failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "current swing_height: %f",
                    swing_height);
      }

      if (arg_pair.first == "get_stand_height") {
        float stand_height = NAN;
        auto ret = client_.GetStandHeight(stand_height);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetStandHeight failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "current stand_height: %f",
                    stand_height);
      }

      if (arg_pair.first == "get_phase") {
        std::vector<float> phase;
        auto ret = client_.GetPhase(phase);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "GetPhase failed");
          continue;
        }
        std::stringstream ss;
        ss << "current phase: (";
        for (const auto &p : phase) {
          ss << p << ", ";
        }
        ss << ")";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }

      if (arg_pair.first == "set_fsm_id") {
        int fsm_id = std::stoi(arg_pair.second);
        auto ret = client_.SetFsmId(fsm_id);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetFsmId failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set fsm_id to %d", fsm_id);
      }

      if (arg_pair.first == "set_balance_mode") {
        int balance_mode = std::stoi(arg_pair.second);
        auto ret = client_.SetBalanceMode(balance_mode);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetBalanceMode failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set balance_mode to %d", balance_mode);
      }

      if (arg_pair.first == "set_swing_height") {
        float swing_height = std::stof(arg_pair.second);
        auto ret = client_.SetSwingHeight(swing_height);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetSwingHeight failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set swing_height to %f", swing_height);
      }

      if (arg_pair.first == "set_stand_height") {
        float stand_height = std::stof(arg_pair.second);
        auto ret = client_.SetStandHeight(stand_height);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetStandHeight failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set stand_height to %f", stand_height);
      }

      if (arg_pair.first == "set_velocity") {
        std::vector<float> param = stringToFloatVector(arg_pair.second);
        auto param_size = param.size();
        float vx = NAN;
        float vy = NAN;
        float omega = NAN;
        float duration = NAN;
        if (param_size == 3) {
          vx = param.at(0);
          vy = param.at(1);
          omega = param.at(2);
          duration = 1.F;
        } else if (param_size == 4) {
          vx = param.at(0);
          vy = param.at(1);
          omega = param.at(2);
          duration = param.at(3);
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid param size for method SetVelocity: %zu",
                       param_size);
          continue;
        }

        auto ret = client_.SetVelocity(vx, vy, omega, duration);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetVelocity failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set velocity to %s",
                    arg_pair.second.c_str());
      }

      if (arg_pair.first == "damp") {
        auto ret = client_.Damp();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "Damp failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Damp command sent");
      }

      if (arg_pair.first == "start") {
        auto ret = client_.Start();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "Start failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Start command sent");
      }

      if (arg_pair.first == "squat") {
        auto ret = client_.Squat();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "Squat failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Squat command sent");
      }

      if (arg_pair.first == "sit") {
        auto ret = client_.Sit();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "Sit failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Sit command sent");
      }

      if (arg_pair.first == "stand_up") {
        auto ret = client_.StandUp();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "StandUp failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "StandUp command sent");
      }

      if (arg_pair.first == "zero_torque") {
        auto ret = client_.ZeroTorque();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "ZeroTorque failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "ZeroTorque command sent");
      }

      if (arg_pair.first == "stop_move") {
        auto ret = client_.StopMove();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "StopMove failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "StopMove command sent");
      }

      if (arg_pair.first == "high_stand") {
        auto ret = client_.HighStand();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "HighStand failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "HighStand command sent");
      }

      if (arg_pair.first == "low_stand") {
        auto ret = client_.LowStand();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "LowStand failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "LowStand command sent");
      }

      if (arg_pair.first == "balance_stand") {
        auto ret = client_.BalanceStand();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "BalanceStand failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "BalanceStand command sent");
      }

      if (arg_pair.first == "continous_gait") {
        bool flag = false;
        if (arg_pair.second == "true") {
          flag = true;
        } else if (arg_pair.second == "false") {
          flag = false;
        } else {
          RCLCPP_ERROR(this->get_logger(), "invalid argument: %s",
                       arg_pair.second.c_str());
          continue;
        }
        auto ret = client_.ContinuousGait(flag);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "ContinuousGait failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "ContinuousGait set to %s",
                    arg_pair.second.c_str());
      }

      if (arg_pair.first == "switch_move_mode") {
        bool flag = false;
        if (arg_pair.second == "true") {
          flag = true;
        } else if (arg_pair.second == "false") {
          flag = false;
        } else {
          RCLCPP_ERROR(this->get_logger(), "invalid argument: %s",
                       arg_pair.second.c_str());
          continue;
        }
        auto ret = client_.SwitchMoveMode(flag);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SwitchMoveMode failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "SwitchMoveMode set to %s",
                    arg_pair.second.c_str());
      }

      if (arg_pair.first == "move") {
        std::vector<float> param = stringToFloatVector(arg_pair.second);
        auto param_size = param.size();
        float vx = NAN;
        float vy = NAN;
        float omega = NAN;
        if (param_size == 3) {
          vx = param.at(0);
          vy = param.at(1);
          omega = param.at(2);
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid param size for method SetVelocity: %zu",
                       param_size);
          continue;
        }
        auto ret = client_.Move(vx, vy, omega);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "Move failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Move command sent with params: %s",
                    arg_pair.second.c_str());
      }

      if (arg_pair.first == "set_task_id") {
        int task_id = std::stoi(arg_pair.second);
        auto ret = client_.SetTaskId(task_id);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetTaskId failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set task_id to %d", task_id);
      }

      if (arg_pair.first == "shake_hand") {
        auto ret = client_.ShakeHand(0);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "ShakeHand start failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Shake hand starts! Waiting for 10 s for ending");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        auto ret_end = client_.ShakeHand(1);
        if (!handleActionError(ret_end)) {
          RCLCPP_ERROR(this->get_logger(), "ShakeHand end failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Shake hand ends!");
      }

      if (arg_pair.first == "wave_hand") {
        auto ret = client_.WaveHand();
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "WaveHand failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "wave hand command sent");
      }

      if (arg_pair.first == "wave_hand_with_turn") {
        auto ret = client_.WaveHand(true);
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "WaveHand with turn failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "wave hand with turn command sent");
      }

      if (arg_pair.first == "set_speed_mode") {
        auto ret = client_.SetSpeedMode(std::stoi(arg_pair.second));
        if (!handleActionError(ret)) {
          RCLCPP_ERROR(this->get_logger(), "SetSpeedMode failed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "set speed mode to %s",
                    arg_pair.second.c_str());
      }

      RCLCPP_INFO(this->get_logger(), "Done processing command: %s",
                  arg_pair.first.c_str());
    }
  }

  static std::vector<float> stringToFloatVector(const std::string &str) {
    std::vector<float> result;
    std::stringstream ss(str);
    float num = NAN;
    while (ss >> num) {
      result.push_back(num);
      // ignore any trailing whitespace
      ss.ignore();
    }
    return result;
  }

  std::map<std::string, std::string> args_;
  unitree::robot::g1::LocoClient client_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);

  std::map<std::string, std::string> args = {};

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.substr(0, 2) == "--") {
      size_t pos = arg.find('=');
      std::string key;
      std::string value;
      if (pos != std::string::npos) {
        key = arg.substr(2, pos - 2);
        value = arg.substr(pos + 1);

        if (value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.length() - 2);
        }
      } else {
        key = arg.substr(2);
        value = "";
      }
      if (args.find(key) != args.end()) {
        args[key] = value;
      } else {
        args.insert({{key, value}});
      }
    }
  }
  if (args.size() == 0) {
    std::cout << "usage: g1_loco_client_example [--get_fsm_id | ...]"
              << std::endl;
    return 0;
  }
  auto node = std::make_shared<UnitreeG1ControlNode>(args);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}