#pragma once
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;
  using ResponsePromise = std::promise<const std::shared_ptr<const Response>>;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr response_sub_;
  std::mutex pending_mutex_;
  std::unordered_map<int64_t, std::shared_ptr<ResponsePromise>>
      pending_promises_;

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))) {
    response_sub_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1),
        [this](const std::shared_ptr<const Response> data) {
          std::shared_ptr<ResponsePromise> promise;
          const auto identity_id = data->header.identity.id;
          {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            auto it = pending_promises_.find(identity_id);
            if (it != pending_promises_.end()) {
              promise = std::move(it->second);
              pending_promises_.erase(it);
            }
          }
          if (promise) {
            promise->set_value(data);
          }
        });
  }

  int32_t Call(Request req, nlohmann::json& js) {
    auto response_promise = std::make_shared<ResponsePromise>();
    auto response_future = response_promise->get_future();
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;

    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_promises_[identity_id] = response_promise;
    }

    req_puber_->publish(req);
    auto status = response_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      auto response_shared = response_future.get();
      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_promises_.erase(identity_id);
      }
      const Response& response = *response_shared;
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
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_promises_.erase(identity_id);
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
