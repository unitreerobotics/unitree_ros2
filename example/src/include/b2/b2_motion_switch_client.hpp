#pragma once

#include <utility>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <string>

#include "b2_base_client.hpp"
#include "common/ut_errror.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
 
namespace unitree::robot::b2 {
 
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
    try {
      unitree_api::msg::Request req;
      req.header.identity.api_id = MOTION_SWITCHER_API_ID_CHECK_MODE;

      nlohmann::json js_res;
      auto result = base_client_.Call(req, js_res);
      if (result == 0) {
        auto parsed_name = js_res.at("name").get<std::string>();
        auto parsed_form = js_res.at("form").get<std::string>();
        name = std::move(parsed_name);
        form = std::move(parsed_form);
      }
      return result;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
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
    try {
      unitree_api::msg::Request req;
      req.header.identity.api_id = MOTION_SWITCHER_API_ID_GET_SILENT;

      nlohmann::json js_res;
      auto result = base_client_.Call(req, js_res);
      if (result == 0) {
        silent = js_res.at("silent").get<bool>();
      }
      return result;
    } catch (const nlohmann::json::exception&) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }
};

}  // namespace unitree::robot::b2
