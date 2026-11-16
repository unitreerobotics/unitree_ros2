#pragma once
#include <cstdint>
#include <utility>
#include <atomic>
#include <mutex>
#include <iostream>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
// Galactic and later ship rclcpp/version.h together with callback groups that
// are not added to executors automatically and Executor::add_callback_group().
// Foxy has neither; there the client keeps upstream's behaviour of spinning the
// whole node in its private executor, so the caller must not spin that node too.
#if __has_include(<rclcpp/version.h>)
#define UNITREE_B2_CLIENT_OWN_CALLBACK_GROUP 1
#endif
#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request  = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_{nullptr};
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr persistent_sub_;

  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::SubscriptionOptions sub_opts_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<int64_t> timeout_ns_{std::chrono::nanoseconds(std::chrono::seconds(5)).count()};

  std::atomic<uint64_t> current_request_id_{0};
  std::atomic<int64_t> current_api_id_{0};
  // Held for the whole of a call: one request in flight per client.
  std::timed_mutex call_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::shared_ptr<const Response> received_response_;
  bool response_ready_{false};

public:
  BaseClient(rclcpp::Node* node,
             const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)) {
    InitRosComm();
  }

  ~BaseClient() {
    stop_requested_.store(true);
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // Per-call timeout, including DDS discovery; the default is five seconds.
  void SetTimeout(std::chrono::nanoseconds timeout) {
    timeout_ns_.store(timeout.count());
  }

  int32_t Call(Request req, nlohmann::json& js) {
    return CallImpl(std::move(req), &js);
  }

  int32_t Call(Request req) {
    return CallImpl(std::move(req), nullptr);
  }

private:
  int32_t CallImpl(Request req, nlohmann::json* js) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::nanoseconds(timeout_ns_.load());
    // Calls are serialized. A call waits for the call in flight only until its
    // own deadline and then returns UT_ROBOT_TASK_NOT_PUBLISHED.
    std::unique_lock<std::timed_mutex> call_lock(call_mutex_, std::defer_lock);
    if (!call_lock.try_lock_until(deadline)) {
      return UT_ROBOT_TASK_NOT_PUBLISHED;
    }
    ResetState();

    req.header.identity.id = unitree::common::NextRequestId();
    current_api_id_.store(req.header.identity.api_id);
    current_request_id_.store(req.header.identity.id);

    // The request and response topics are volatile: a request published
    // before the server's reader matches, or a response published before our
    // reader matches the server's writer, is lost. Discovery can exceed the
    // constructor's bounded wait (slow links) and restarts after a server
    // restart, so wait for both matches within this call's deadline.
    if (!WaitForMatch(deadline) || std::chrono::steady_clock::now() >= deadline) {
      // Check immediately before publication. Scheduling and middleware
      // blocking cannot be bounded by this application-side check.
      return UT_ROBOT_TASK_NOT_PUBLISHED;
    }
    req_puber_->publish(req);
    std::unique_lock<std::mutex> response_lock(response_mutex_);

    if (response_cv_.wait_until(response_lock, deadline, [this]() {
        return response_ready_;
    })) {

        if (!received_response_) {
            return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }

        if (received_response_->header.status.code != 0) {
            return received_response_->header.status.code;
        }
        if (js == nullptr) {
            return UT_ROBOT_SUCCESS;
        }
        if (received_response_->data.empty()) {
            *js = nlohmann::json::object();
            return UT_ROBOT_SUCCESS;
        }

        try {
            if (received_response_->data.find('\0') != std::string::npos) {
                return UT_ROBOT_TASK_UNKNOWN_ERROR;
            }
            *js = nlohmann::json::parse(received_response_->data);
            return UT_ROBOT_SUCCESS;
        } catch (const nlohmann::detail::exception& e) {
            return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }
    } else {
        // Published without a response in time: the outcome is unknown.
        return UT_ROBOT_TASK_TIMEOUT;
    }
  }

  // Request writer QoS. These are discrete API requests, so the history keeps
  // its depth of ten and unacknowledged requests stay queued in order. Lifespan
  // of five seconds (the default timeout): the writer stops retransmitting a
  // request five seconds after publishing it.
  // This cannot retract a packet already in the network or a request already
  // received by the server. It is not a delivery or execution deadline.
  // Neither policy is checked when endpoints match. The lifespan is fixed and
  // does not follow SetTimeout(). A timeout does not cancel the request.
  static rclcpp::QoS RequestQoS() {
    return rclcpp::QoS(10).lifespan(rclcpp::Duration(std::chrono::seconds(5)));
  }

  void InitRosComm() {
    req_puber_ = node_->create_publisher<Request>(topic_name_request_, RequestQoS());
    // Only the response callback belongs to the private executor. The caller
    // may already spin this node, or add other B2 clients to the same node.
#ifdef UNITREE_B2_CLIENT_OWN_CALLBACK_GROUP
    sub_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
#else
    sub_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
#endif
    sub_opts_.callback_group = sub_group_;

    persistent_sub_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(20),
        [this](const std::shared_ptr<const Response> data) {
          this->OnResponseReceived(data);
        },
        sub_opts_);

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
#ifdef UNITREE_B2_CLIENT_OWN_CALLBACK_GROUP
    executor_->add_callback_group(sub_group_, node_->get_node_base_interface());
#else
    executor_->add_node(node_->get_node_base_interface());
#endif
    spin_thread_ = std::thread([this]() {
      // cancel() can precede the thread entering spin(), which resets the
      // executor's spinning flag. Bound each wait and retain our stop request
      // independently so immediate destruction cannot leave join() blocked.
      while (!stop_requested_.load() && rclcpp::ok(node_->get_node_options().context())) {
        executor_->spin_once(std::chrono::milliseconds(50));
      }
    });

    WaitForConnection();
  }

  // Matched DDS endpoints, not ROS graph entries: the graph can list a
  // remote endpoint before this publisher or subscription has matched it.
  bool Matched() const {
    return req_puber_->get_subscription_count() > 0 &&
           persistent_sub_->get_publisher_count() > 0;
  }

  bool WaitForMatch(std::chrono::steady_clock::time_point deadline) const {
    while (!Matched()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }

  // Bounded startup wait kept from the previous behaviour; Call() waits again.
  void WaitForConnection() {
    WaitForMatch(std::chrono::steady_clock::now() + std::chrono::milliseconds(500));
  }

  void OnResponseReceived(const std::shared_ptr<const Response> data) {
    std::lock_guard<std::mutex> lk(response_mutex_);
    const uint64_t expected_id = current_request_id_.load();
    const uint64_t resp_id = data->header.identity.id;

    if (expected_id != 0 && resp_id == expected_id &&
        data->header.identity.api_id == current_api_id_.load() && !response_ready_) {
            received_response_ = data;
            response_ready_ = true;
        response_cv_.notify_one();
    }
  }

  void ResetState() {
    current_request_id_.store(0);
    current_api_id_.store(0);
    {
        std::lock_guard<std::mutex> lk(response_mutex_);
        received_response_.reset();
        response_ready_ = false;
    }
  }
};
