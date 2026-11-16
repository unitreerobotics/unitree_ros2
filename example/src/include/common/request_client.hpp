// BSD 3-Clause License
//
// Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
// Copyright 2026 Yusuf Guenena
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

// Request/response client for the Unitree API topics.
//
// One response subscription lives for the whole client lifetime, so responses
// are never lost to a reader created at call time. Each call registers its
// (identity.id, api_id) before publishing; the subscription callback completes
// only the matching pending call, once. Late responses for calls that already
// timed out, duplicates and responses for other clients are dropped.
//
// One call is in flight per client. Concurrent callers wait for the call in
// flight, but never past their own deadline: a call that cannot start before
// its deadline returns UT_ROBOT_TASK_NOT_PUBLISHED without publishing anything.
// Waiting callers are not served in a guaranteed order. API requests are not
// idempotent and are never retried by this client; see RequestQoS() for what
// the writer retains. A request can still arrive after its call has returned.
//
// UT_ROBOT_TASK_NOT_PUBLISHED means the request was never handed to DDS.
// UT_ROBOT_TASK_TIMEOUT means it was handed to DDS and no matching response
// arrived in time. That outcome is unknown: the server may have executed the
// request, may still receive it later, and its response may be lost or late.
//
// As before, the caller must spin the node passed to the constructor while
// Call() waits, from a thread other than the one calling Call().
namespace unitree
{
namespace ros2
{

class RequestClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;
  using Key = std::pair<int64_t, int64_t>;  // identity.id, identity.api_id

  struct PendingCalls
  {
    std::mutex mutex;
    std::map<Key, std::promise<Response::ConstSharedPtr>> calls;
  };

  // Copies share the same DDS writer and response state. They must also share
  // single-flight coordination, rather than independently replacing samples.
  struct CallControl
  {
    std::atomic<int64_t> timeout_ns{
      std::chrono::nanoseconds(std::chrono::seconds(5)).count()};
    std::timed_mutex mutex;
  };

  rclcpp::Node * node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  // Shared with the subscription callback so it never touches this object.
  std::shared_ptr<PendingCalls> pending_;
  rclcpp::Subscription<Response>::SharedPtr resp_suber_;
  std::shared_ptr<CallControl> control_{std::make_shared<CallControl>()};

  // Request writer QoS. The history depth is the owner's: one for the generic
  // clients, ten for legacy query paths, as they had before. With a depth of
  // one a later call replaces a request the server has not acknowledged; with
  // more, earlier unacknowledged requests stay queued ahead of it. Lifespan of
  // five seconds (the default timeout): the writer stops retransmitting a
  // request five seconds after publishing it.
  // This cannot retract a packet already in the network or a request already
  // received by the server. It is not a delivery or execution deadline.
  // Neither policy is checked when endpoints match. The lifespan is fixed and
  // does not follow SetTimeout(). A timeout does not cancel the request.
  static rclcpp::QoS RequestQoS(size_t history)
  {
    return rclcpp::QoS(history).lifespan(rclcpp::Duration(std::chrono::seconds(5)));
  }

  // A request published before the server's reader is matched is lost, and a
  // response published before our reader is matched to the server's writer is
  // lost. Both are volatile topics, so wait for discovery within the deadline.
  bool WaitForMatch(std::chrono::steady_clock::time_point deadline) const
  {
    while (req_puber_->get_subscription_count() == 0 ||
      resp_suber_->get_publisher_count() == 0)
    {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }

public:
  RequestClient(
    rclcpp::Node * node, const std::string & topic_name_request,
    std::string topic_name_response, size_t request_history = 1)
  : node_(node),
    topic_name_request_(topic_name_request),
    topic_name_response_(std::move(topic_name_response)),
    req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    RequestQoS(request_history))),
    pending_(std::make_shared<PendingCalls>())
  {
    // This reader lives as long as the client and receives every response on
    // the shared topic, whether or not the node is being spun. The depth holds
    // concurrent bursts until the executor can correlate them and bounds what an
    // idle or stalled executor retains. The oldest sample is dropped first, and
    // a pending call's response is the newest. Reliability and durability
    // remain unchanged.
    std::weak_ptr<PendingCalls> weak_pending = pending_;
    resp_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1000),
      [weak_pending](const Response::ConstSharedPtr data) {
        auto pending = weak_pending.lock();
        if (!pending) {
          return;
        }
        std::promise<Response::ConstSharedPtr> promise;
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          auto it = pending->calls.find(
                Key(data->header.identity.id, data->header.identity.api_id));
          if (it == pending->calls.end()) {
            return;    // Not ours, duplicate, or arrived after a timeout.
          }
          promise = std::move(it->second);
          pending->calls.erase(it);
        }
        promise.set_value(data);
        });
  }

  // Per-call timeout, including DDS discovery; the default is five seconds.
  void SetTimeout(std::chrono::nanoseconds timeout)
  {
    control_->timeout_ns.store(timeout.count());
  }

  int32_t Call(Request req, nlohmann::json & js)
  {
    return CallImpl(std::move(req), &js);
  }

  int32_t Call(Request req)
  {
    return CallImpl(std::move(req), nullptr);
  }

private:
  int32_t CallImpl(Request req, nlohmann::json * js)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::nanoseconds(control_->timeout_ns.load());
    // Wait for the call in flight, within this call's own deadline.
    std::unique_lock<std::timed_mutex> in_flight(control_->mutex, std::defer_lock);
    if (!in_flight.try_lock_until(deadline)) {
      return UT_ROBOT_TASK_NOT_PUBLISHED;
    }
    req.header.identity.id = unitree::common::NextRequestId();
    const Key key(req.header.identity.id, req.header.identity.api_id);

    std::future<Response::ConstSharedPtr> response_future;
    {
      std::lock_guard<std::mutex> lock(pending_->mutex);
      response_future = pending_->calls[key].get_future();
    }

    try {
      if (!WaitForMatch(deadline) || std::chrono::steady_clock::now() >= deadline) {
        // Check immediately before publication. Scheduling and middleware
        // blocking cannot be bounded by this application-side check.
        std::lock_guard<std::mutex> lock(pending_->mutex);
        pending_->calls.erase(key);
        return UT_ROBOT_TASK_NOT_PUBLISHED;
      }
      req_puber_->publish(req);
    } catch (...) {
      // Middleware discovery/publication failures must not retain a promise
      // for a request whose caller has already unwound.
      std::lock_guard<std::mutex> lock(pending_->mutex);
      pending_->calls.erase(key);
      throw;
    }
    auto status = response_future.wait_until(deadline);
    if (status != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(pending_->mutex);
      // Removing the map entry does not make the future ready: the callback
      // moves the promise out before setting it. Never call get() after a
      // failed wait, even if the callback already claimed the promise.
      pending_->calls.erase(key);
      return UT_ROBOT_TASK_TIMEOUT;
    }

    // Keep the message alive for the whole parse; get() returns a temporary.
    const Response::ConstSharedPtr response_ptr = response_future.get();
    const Response & response = *response_ptr;
    if (response.header.status.code != 0) {
      std::cout << "error code: " << response.header.status.code << std::endl;
      return response.header.status.code;
    }
    if (js != nullptr) {
      // JSON cannot contain literal NUL bytes. A C-string parser would hide
      // any bytes following one and could accept a corrupt response.
      if (response.data.find('\0') != std::string::npos) {
        return UT_ROBOT_TASK_UNKNOWN_ERROR;
      }
      try {
        *js = nlohmann::json::parse(response.data);
      } catch (const nlohmann::json::exception &) {
        return UT_ROBOT_TASK_UNKNOWN_ERROR;
      }
    }
    return UT_ROBOT_SUCCESS;
  }
};

}  // namespace ros2
}  // namespace unitree
