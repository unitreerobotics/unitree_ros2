#pragma once
#include <cstdint>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))) {}

  int32_t Call(Request req, nlohmann::json& js) {
    std::promise<const std::shared_ptr<const Response>> response_promise;
    auto response_future = response_promise.get_future();
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;

    auto req_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1),
        [&response_promise,
         identity_id](const std::shared_ptr<const Response> data) {
          if (data->header.identity.id == identity_id) {
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
      try {
        js = nlohmann::json::parse(response.data.data());
      } catch (nlohmann::detail::exception& e) {
      }
      return UT_ROBOT_SUCCESS;
    }
    if (status == std::future_status::timeout) {
      return UT_ROBOT_TASK_TIMEOUT;
    }
    return UT_ROBOT_TASK_UNKNOWN_ERROR;
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js);
  }
};