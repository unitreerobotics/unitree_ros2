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
 
namespace unitree::robot::g1 {
 
const int32_t MOTION_SWITCHER_API_ID_CHECK_MODE = 1001;
const int32_t MOTION_SWITCHER_API_ID_SELECT_MODE = 1002;
const int32_t MOTION_SWITCHER_API_ID_RELEASE_MODE = 1003;
const int32_t MOTION_SWITCHER_API_ID_SET_SILENT = 1004;
const int32_t MOTION_SWITCHER_API_ID_GET_SILENT = 1005;

class MotionSwitchClient {
  rclcpp::Node *node_;
  BaseClient base_client_;

 public:
  explicit MotionSwitchClient(rclcpp::Node *node)
      : node_(node),
        base_client_(node_, "/api/motion_switcher/request", "/api/motion_switcher/response") {}

  int32_t CheckMode(std::string &form, std::string &name) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = MOTION_SWITCHER_API_ID_CHECK_MODE;

    nlohmann::json js_res;
    auto result = base_client_.Call(req, js_res);
    if (result == 0) {
      js_res["name"].get_to(name);
      js_res["form"].get_to(form);
    }
    return result;
  }

  int32_t SelectMode(const std::string &name_or_alias) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = MOTION_SWITCHER_API_ID_SELECT_MODE;
    nlohmann::json js;
    js["name"] = name_or_alias;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t ReleaseMode() {
    unitree_api::msg::Request req;
    req.header.identity.api_id = MOTION_SWITCHER_API_ID_RELEASE_MODE;
    return base_client_.Call(req);
  }

  int32_t SetSilent(bool silent) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = MOTION_SWITCHER_API_ID_SET_SILENT;
    nlohmann::json js;
    js["silent"] = silent;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t GetSilent(bool &silent) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = MOTION_SWITCHER_API_ID_GET_SILENT;

    nlohmann::json js_res;
    auto result = base_client_.Call(req, js_res);
    if (result == 0) {
      js_res["silent"].get_to(silent);
    }
    return result;
  }
};

}  // namespace unitree::robot::g1
