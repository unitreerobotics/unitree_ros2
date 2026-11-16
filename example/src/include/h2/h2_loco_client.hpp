#pragma once

#include <utility>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "patch.hpp"
#include "common/b2_base_client.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"

namespace unitree::robot::h2 {

static constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 7001;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_PHASE = 7006;  // deprecated
static constexpr int32_t ROBOT_API_ID_LOCO_GET_ARM_SDK_STATUS = 7007;
static constexpr int32_t ROBOT_API_ID_LOCO_GET_AVAILABLE_FSM_IDS = 7008;

static constexpr int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 7101;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_SPEED_MODE = 7107;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_PUNCH_API = 7108;
static constexpr int32_t ROBOT_API_ID_LOCO_SET_ARM_SDK_STATUS = 7109;

class H2LocoClient {
  using Request = unitree_api::msg::Request;

  rclcpp::Node* node_{nullptr};
  BaseClient base_client_;

 public:
  explicit H2LocoClient(rclcpp::Node* node)
      : node_(node),
        base_client_(node_, "/api/sport/request", "/api/sport/response") {}

  int32_t GetFsmId(int& fsm_id) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_FSM_ID;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) fsm_id = js.at("data").get<int>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetFsmMode(int& fsm_mode) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_FSM_MODE;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) fsm_mode = js.at("data").get<int>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetBalanceMode(int& balance_mode) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_BALANCE_MODE;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) balance_mode = js.at("data").get<int>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetSwingHeight(float& swing_height) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_SWING_HEIGHT;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) swing_height = js.at("data").get<float>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetStandHeight(float& stand_height) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_STAND_HEIGHT;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) stand_height = js.at("data").get<float>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetPhase(std::vector<float>& phase) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_PHASE;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) phase = js.at("data").get<std::vector<float>>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetArmSdkStatus(bool& arm_sdk_status) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_ARM_SDK_STATUS;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) arm_sdk_status = js.at("data").get<bool>();
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t GetAvailableFsmIds(std::vector<int>& ids, std::vector<std::string>& names) {
    try {
      Request req;
      req.header.identity.api_id = ROBOT_API_ID_LOCO_GET_AVAILABLE_FSM_IDS;
      nlohmann::json js;
      int32_t ret = base_client_.Call(req, js);
      if (ret == 0) {
        if (!js.is_object()) return UT_ROBOT_TASK_UNKNOWN_ERROR;
        std::vector<int> parsed_ids;
        std::vector<std::string> parsed_names;
        if (js.contains("ids")) js.at("ids").get_to(parsed_ids);
        if (js.contains("names")) js.at("names").get_to(parsed_names);
        ids = std::move(parsed_ids);
        names = std::move(parsed_names);
      }
      return ret;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  int32_t SetFsmId(int fsm_id) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
    nlohmann::json js;
    js["data"] = fsm_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetBalanceMode(int balance_mode) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_BALANCE_MODE;
    nlohmann::json js;
    js["data"] = balance_mode;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetSwingHeight(float swing_height) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_SWING_HEIGHT;
    nlohmann::json js;
    js["data"] = swing_height;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetStandHeight(float stand_height) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_STAND_HEIGHT;
    nlohmann::json js;
    js["data"] = stand_height;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.F) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
    nlohmann::json js;
    js["velocity"] = std::vector<float>{vx, vy, omega};
    js["duration"] = duration;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetTaskId(int task_id) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_ARM_TASK;
    nlohmann::json js;
    js["data"] = task_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetSpeedMode(int speed_mode) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_SPEED_MODE;
    nlohmann::json js;
    js["data"] = speed_mode;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetPunchApi(const std::vector<float>& punch_api) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_PUNCH_API;
    nlohmann::json js;
    js["data"] = punch_api;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t SetArmSdkStatus(bool arm_sdk_status) {
    Request req;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_ARM_SDK_STATUS;
    nlohmann::json js;
    js["data"] = arm_sdk_status;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t Damp() { return SetFsmId(1); }
  int32_t Start() { return SetFsmId(601); }
  int32_t Squat() { return SetFsmId(2); }
  int32_t Sit() { return SetFsmId(3); }
  int32_t StandUp() { return SetFsmId(4); }
  int32_t ZeroTorque() { return SetFsmId(0); }
  int32_t StopMove() { return SetVelocity(0.F, 0.F, 0.F); }

  int32_t HighStand() {
    return SetStandHeight(static_cast<float>(std::numeric_limits<uint32_t>::max()));
  }

  int32_t LowStand() { return SetStandHeight(std::numeric_limits<uint32_t>::min()); }

  int32_t Move(float vx, float vy, float vyaw, bool continuous_move) {
    return SetVelocity(vx, vy, vyaw, continuous_move ? 864000.F : 1.F);
  }

  int32_t Move(float vx, float vy, float vyaw) { return Move(vx, vy, vyaw, continuous_move_); }

  int32_t BalanceStand() { return SetBalanceMode(0); }

  int32_t ContinuousGait(bool flag) { return SetBalanceMode(flag ? 1 : 0); }

  int32_t SwitchMoveMode(bool flag) {
    continuous_move_ = flag;
    return 0;
  }

  int32_t WaveHand(bool turn_flag = false) { return SetTaskId(turn_flag ? 1 : 0); }

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

  int32_t EnableArmSDK() { return SetArmSdkStatus(true); }
  int32_t DisableArmSDK() { return SetArmSdkStatus(false); }

 private:
  bool continuous_move_{false};
  bool first_shake_hand_stage_{true};
};

}  // namespace unitree::robot::h2

