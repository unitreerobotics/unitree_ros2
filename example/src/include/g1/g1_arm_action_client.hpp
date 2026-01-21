#pragma once

#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <string>

#include "base_client.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

#define UT_DECL_ERR(name, code, desc) \
  const int32_t name = code;          \
  const constexpr char* name##_DESC = desc;

namespace unitree::robot::g1 {

UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_ARMSDK, 7400,
            "The topic rt/armsdk is occupied.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_HOLDING, 7401,
            "The arm is holding. Expecting release action(99) or the same last "
            "action id.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID, 7402,
            "Invalid action id.")
// The actions are only supported in fsm id {500, 501, 801};
// You can subscribe the topic rt/sportmodestate to check the fsm id.
// And in the state 801, the actions are only supported in the fsm mode {0, 3}.
// See
// https://support.unitree.com/home/en/G1_developer/sport_services_interface#Expert%20interface
UT_DECL_ERR(
    UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID, 7404,
    "Invalid fsm id.\n"
    "The actions are only supported in fsm id {500, 501, 801}.\n"
    "You can subscribe the topic rt/sportmodestate to check the fsm id.\n"
    "And in the state 801, the actions are only supported in the fsm "
    "mode {0, 3}.\n"
    "See "
    "https://support.unitree.com/home/en/G1_developer/"
    "sport_services_interface#Expert%20interface");

const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION = 7106;
const int32_t ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST = 7107;

class G1ArmActionClient {
  rclcpp::Node* node_;
  BaseClient base_client_;

 public:
  explicit G1ArmActionClient(rclcpp::Node* node)
      : node_(node),
        base_client_(node_, "/api/arm/request", "/api/arm/response") {}

  int32_t ExecuteAction(int32_t action_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION;
    nlohmann::json js;
    js["data"] = action_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t GetActionList(std::string& data) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST;

    nlohmann::json js_res;
    auto result = base_client_.Call(req, js_res);
    if (result == 0) {
      data = js_res.dump();
    }
    return result;
  }
};

}  // namespace unitree::robot::g1
