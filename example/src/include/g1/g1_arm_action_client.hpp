#pragma once

#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <string>

#include "nlohmann/json.hpp"
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
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID, 7404, "Invalid fsm id.")

const int32_t ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION = 7106;
const int32_t ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST = 7107;

class G1ArmActionClient {
  rclcpp::Node* node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;

 public:
  explicit G1ArmActionClient(rclcpp::Node* node)
      : node_(node),
        req_puber_(node_->create_publisher<unitree_api::msg::Request>(
            "/api/arm/request", 10)) {}

  template <typename Request, typename Response>
  int32_t Call(const Request& req, nlohmann::json& js) {
    std::promise<const std::shared_ptr<const Response>> response_promise;
    auto response_future = response_promise.get_future();
    const auto api_id = req.header.identity.api_id;

    auto req_suber_ = node_->create_subscription<Response>(
        "/api/arm/response", rclcpp::QoS(1),
        [&response_promise, api_id](const std::shared_ptr<const Response> data) { 
          if (data->header.identity.api_id == api_id) {
            response_promise.set_value(data);
          }
        });

    req_puber_->publish(req);
    auto status = response_future.wait_for(std::chrono::seconds(5));

    Response response;
    if (status == std::future_status::ready) {
      response = *response_future.get();
      if (response.header.status.code != 0) {
        std::cout << "error code: " << response.header.status.code << std::endl;
        return response.header.status.code;
      }
      js = nlohmann::json::parse(response.data.data());
      return 0;
      std::cout << "task finish." << std::endl;
    }
    if (status == std::future_status::timeout) {
      std::cout << "task timeout." << std::endl;
      return -1;
    }
    std::cout << "task error." << std::endl;
    return -2;
  }

  static int64_t GetSystemUptimeInNanoseconds() {
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
  }

  int32_t ExecuteAction(int32_t action_id) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION;
    req.header.identity.id = GetSystemUptimeInNanoseconds();
    nlohmann::json js;
    js["action_id"] = action_id;
    req.parameter = js.dump();
    nlohmann::json js_res;
    auto result = Call<unitree_api::msg::Request, unitree_api::msg::Response>(
        req, js_res);
    return result;
  }

  int32_t GetActionList(std::string& data) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST;
    req.header.identity.id = GetSystemUptimeInNanoseconds();
    nlohmann::json js_res;
    auto result = Call<unitree_api::msg::Request, unitree_api::msg::Response>(
        req, js_res);
    if (result == 0) {
      data = js_res.dump();
    }
    return result;
  }
};

}  // namespace unitree::robot::g1
