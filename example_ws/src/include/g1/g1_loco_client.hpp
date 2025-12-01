#ifndef __UT_ROBOT_G1_LOCO_CLIENT_HPP__
#define __UT_ROBOT_G1_LOCO_CLIENT_HPP__

#include <cstdint>
#include <iostream>
#include <limits>
#include <rclcpp/node.hpp>
#include <string>

#include "base_client.hpp"
#include "common/ut_errror.hpp"
#include "detail/exceptions.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

const int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 7001;
const int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002;
const int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003;
const int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004;
const int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005;
const int32_t ROBOT_API_ID_LOCO_GET_PHASE = 7006;  // deprecated

const int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 7101;
const int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102;
const int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103;
const int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104;
const int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;
const int32_t ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106;
const int32_t ROBOT_API_ID_LOCO_SET_SPEED_MODE = 7107;

namespace unitree::robot::g1 {

UT_DECL_ERR(UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE, 7301,
            "LocoState not available.")
UT_DECL_ERR(UT_ROBOT_LOCO_ERR_INVALID_FSM_ID, 7302, "Invalid fsm id.")
UT_DECL_ERR(UT_ROBOT_LOCO_ERR_INVALID_TASK_ID, 7303, "Invalid task id.")

class LocoClient {
  rclcpp::Node* node_;
  BaseClient base_client_;

 public:
  explicit LocoClient(rclcpp::Node* node)
      : node_(node),
        base_client_(node_, "/api/sport/request", "/api/sport/response") {}

  /*Low Level API Call*/
  int32_t GetFsmId(int& fsm_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_FSM_ID;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      js["data"].get_to(fsm_id);
    }

    return ret;
  }

  int32_t GetFsmMode(int& fsm_mode) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_FSM_MODE;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      js["data"].get_to(fsm_mode);
    }

    return ret;
  }

  int32_t GetBalanceMode(int& balance_mode) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_BALANCE_MODE;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      js["data"].get_to(balance_mode);
    }

    return ret;
  }

  int32_t GetSwingHeight(float& swing_height) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_SWING_HEIGHT;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      std::cout << js.dump() << std::endl;
      js["data"].get_to(swing_height);
    } else {
      std::cerr << "Failed to connect to robot." << std::endl;
    }

    return ret;
  }

  int32_t GetStandHeight(float& stand_height) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_STAND_HEIGHT;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      js["data"].get_to(stand_height);
    }

    return ret;
  }

  int32_t GetPhase(std::vector<float>& phase) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_PHASE;
    nlohmann::json js;
    int32_t ret = base_client_.Call(req, js);
    if (ret == 0) {
      js["data"].get_to(phase);
    }

    return ret;
  }

  int32_t SetFsmId(int fsm_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
    nlohmann::json js;
    js["data"] = fsm_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetBalanceMode(int balance_mode) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_BALANCE_MODE;
    nlohmann::json js;
    js["data"] = balance_mode;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetSwingHeight(float swing_height) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_SWING_HEIGHT;
    nlohmann::json js;
    js["data"] = swing_height;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetStandHeight(float stand_height) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_STAND_HEIGHT;
    nlohmann::json js;
    js["data"] = stand_height;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.F) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
    nlohmann::json js;
    std::vector<float> velocity = {vx, vy, omega};
    js["velocity"] = velocity;
    js["duration"] = duration;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetTaskId(int task_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_ARM_TASK;
    nlohmann::json js;
    js["data"] = task_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetSpeedMode(int speed_mode) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_SPEED_MODE;
    nlohmann::json js;
    js["data"] = speed_mode;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  /*High Level API Call*/
  int32_t Damp() { return SetFsmId(1); }

  int32_t Start() { return SetFsmId(500); }

  int32_t Squat() { return SetFsmId(2); }

  int32_t Sit() { return SetFsmId(3); }

  int32_t StandUp() { return SetFsmId(4); }

  int32_t ZeroTorque() { return SetFsmId(0); }

  int32_t StopMove() { return SetVelocity(0.F, 0.F, 0.F); }

  int32_t HighStand() {
    return SetStandHeight(
        static_cast<float>(std::numeric_limits<uint32_t>::max()));
  }

  int32_t LowStand() {
    return SetStandHeight(std::numeric_limits<uint32_t>::min());
  }

  int32_t Move(float vx, float vy, float vyaw, bool continous_move) {
    return SetVelocity(vx, vy, vyaw, continous_move ? 864000.F : 1.F);
  }

  int32_t Move(float vx, float vy, float vyaw) {
    return Move(vx, vy, vyaw, continous_move_);
  }

  int32_t BalanceStand() { return SetBalanceMode(0); }

  int32_t ContinuousGait(bool flag) { return SetBalanceMode(flag ? 1 : 0); }

  int32_t SwitchMoveMode(bool flag) {
    continous_move_ = flag;
    return 0;
  }

  int32_t WaveHand(bool turn_flag = false) {
    return SetTaskId(turn_flag ? 1 : 0);
  }

  int32_t ShakeHand(int stage = -1) {
    switch (stage) {
      case 0:
        first_shake_hand_stage_ = false;
        return SetTaskId(2);

      case 1:
        first_shake_hand_stage_ = true;
        return SetTaskId(3);

      default:
        first_shake_hand_stage_ = !first_shake_hand_stage_;
        return SetTaskId(first_shake_hand_stage_ ? 3 : 2);
    }
  }

 private:
  bool continous_move_ = false;
  bool first_shake_hand_stage_ = true;
};
}  // namespace unitree::robot::g1

#endif  // __UT_ROBOT_G1_LOCO_CLIENT_HPP__