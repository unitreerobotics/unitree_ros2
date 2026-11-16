// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Single-flight behaviour of an API client: at most one request of a client is
// outstanding at the server, and a call that cannot start before its deadline
// returns UT_ROBOT_TASK_NOT_PUBLISHED without publishing. Built for the generic client
// and, with UNITREE_B2_CLIENT, for the B2 client.
//   in-flight   4 threads x 5 calls, the peer answers each request after 100 ms
//   queued      a call waits behind an unanswered call and must time out on its
//               own deadline without its request ever reaching the peer
//   no-loss     8 threads x 50 calls on a new client, immediate answers: every
//               call succeeds and the peer sees each request exactly once
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#ifdef UNITREE_B2_CLIENT
#include "common/b2_base_client.hpp"
#else
#include "common/base_client.hpp"
#endif

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using unitree_api::msg::Request;
using unitree_api::msg::Response;

// Answers requests from its own thread after a fixed delay, except api 9999,
// which it records but never answers.
class Peer
{
public:
  Peer(const std::string & prefix, std::chrono::milliseconds delay)
  : node_(std::make_shared<rclcpp::Node>("unitree_single_flight_peer")), delay_(delay)
  {
    publisher_ = node_->create_publisher<Response>(prefix + "/response", 100);
    subscription_ = node_->create_subscription<Request>(
      prefix + "/request", 100, [this](Request::ConstSharedPtr request) {
        std::lock_guard<std::mutex> lock(mutex_);
        ids_.push_back(request->header.identity.id);
        apis_.push_back(request->header.identity.api_id);
        if (request->header.identity.api_id == 9999) {return;}
        ++outstanding_;
        max_outstanding_ = std::max(max_outstanding_, outstanding_);
        queue_.push_back({Clock::now() + delay_, request->header.identity});
        cv_.notify_one();
      });
    executor_.add_node(node_);
    spin_ = std::thread([this]() {executor_.spin();});
    responder_ = std::thread([this]() {Respond();});
  }

  ~Peer()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    responder_.join();
    executor_.cancel();
    spin_.join();
  }

  int max_outstanding()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_outstanding_;
  }
  std::vector<int64_t> ids()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ids_;
  }
  std::vector<int64_t> apis()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return apis_;
  }

private:
  struct Due
  {
    Clock::time_point when;
    unitree_api::msg::RequestIdentity identity;
  };

  void Respond()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
      if (queue_.empty()) {
        cv_.wait(lock);
        continue;
      }
      const Due due = queue_.front();
      if (Clock::now() < due.when) {
        cv_.wait_until(lock, due.when);
        continue;
      }
      queue_.pop_front();
      --outstanding_;
      Response response;
      response.header.identity = due.identity;
      response.data = "{\"api\":" + std::to_string(due.identity.api_id) + "}";
      lock.unlock();
      publisher_->publish(response);
      lock.lock();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::chrono::milliseconds delay_;
  rclcpp::Publisher<Response>::SharedPtr publisher_;
  rclcpp::Subscription<Request>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_, responder_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Due> queue_;
  std::vector<int64_t> ids_, apis_;
  int outstanding_{0}, max_outstanding_{0};
  bool stop_{false};
};

int32_t Call(BaseClient & client, int64_t api)
{
  Request request;
  request.header.identity.api_id = api;
  nlohmann::json js;
  const int32_t result = client.Call(request, js);
  if (result == UT_ROBOT_SUCCESS && js.value("api", int64_t{0}) != api) {
    return UT_ROBOT_TASK_UNKNOWN_ERROR;
  }
  return result;
}

bool g_passed = true;
void expect(bool condition, const std::string & what)
{
  std::cout << (condition ? "ok   " : "FAIL ") << what << std::endl;
  g_passed = g_passed && condition;
}

void InFlight(BaseClient & client, Peer & peer, BaseClient * other = nullptr)
{
  std::atomic<int> succeeded{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&, t]() {
        for (int i = 0; i < 5; ++i) {
          auto & handle = other != nullptr && t % 2 ? *other : client;
          if (Call(handle, 3000 + t) == UT_ROBOT_SUCCESS) {++succeeded;}
        }
      });
  }
  for (auto & thread : threads) {thread.join();}
  expect(succeeded.load() == 20, "20 of 20 calls succeed: " + std::to_string(succeeded.load()));
  expect(peer.max_outstanding() == 1,
    "at most one request outstanding at the peer: " + std::to_string(peer.max_outstanding()));
  expect(peer.ids().size() == 20, "peer received 20 requests: " + std::to_string(peer.ids().size()));
}

void Queued(BaseClient & client, Peer & peer)
{
  // The first call is never answered and holds the client for 1 s.
  client.SetTimeout(1s);
  int32_t first = 0;
  std::thread holder([&]() {first = Call(client, 9999);});
  std::this_thread::sleep_for(150ms);
  client.SetTimeout(300ms);
  const auto start = Clock::now();
  const int32_t queued = Call(client, 3001);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now() - start).count();
  holder.join();
  std::this_thread::sleep_for(500ms);  // A request published late would arrive by now.
  const auto apis = peer.apis();
  expect(first == UT_ROBOT_TASK_TIMEOUT, "unanswered call times out: " + std::to_string(first));
  expect(queued == UT_ROBOT_TASK_NOT_PUBLISHED, "queued call is not published: " + std::to_string(queued));
  expect(elapsed >= 250 && elapsed < 700,
    "queued call ends at its own deadline (300 ms): " + std::to_string(elapsed) + " ms");
  expect(std::count(apis.begin(), apis.end(), 3001) == 0,
    "queued call's request never reached the peer");
  // The client is usable once the holder has finished.
  client.SetTimeout(5s);
  expect(Call(client, 3002) == UT_ROBOT_SUCCESS, "next call succeeds");
}

void NoLoss(BaseClient & client, Peer & peer)
{
  std::atomic<int> succeeded{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&, t]() {
        for (int i = 0; i < 50; ++i) {
          if (Call(client, 3000 + t) == UT_ROBOT_SUCCESS) {++succeeded;}
        }
      });
  }
  for (auto & thread : threads) {thread.join();}
  const auto ids = peer.ids();
  const std::set<int64_t> unique(ids.begin(), ids.end());
  expect(succeeded.load() == 400, "400 of 400 calls succeed: " + std::to_string(succeeded.load()));
  expect(ids.size() == 400 && unique.size() == 400,
    "peer received each request once: " + std::to_string(ids.size()) + " received, " +
    std::to_string(unique.size()) + " unique");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "";
  if (mode != "in-flight" && mode != "queued" && mode != "no-loss" && mode != "copies") {
    std::cerr << "usage: " << argv[0] << " in-flight|queued|no-loss|copies" << std::endl;
    return 2;
  }
  const std::string prefix = "/unitree_test/single_flight_" + std::to_string(getpid());
  {
    Peer peer(prefix, mode == "no-loss" ? 0ms : 100ms);
    auto node = std::make_shared<rclcpp::Node>("unitree_single_flight_client");
#ifndef UNITREE_B2_CLIENT
    // The generic client relies on the caller spinning its node.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin([&]() {executor.spin();});
#endif
    {
      BaseClient client(node.get(), prefix + "/request", prefix + "/response");
      if (mode == "in-flight") {
        InFlight(client, peer);
#ifndef UNITREE_B2_CLIENT
      } else if (mode == "copies") {
        auto copy = client;
        InFlight(client, peer, &copy);
#endif
      } else if (mode == "queued") {
        Queued(client, peer);
      } else {
        NoLoss(client, peer);
      }
    }
#ifndef UNITREE_B2_CLIENT
    executor.cancel();
    spin.join();
#endif
  }
  rclcpp::shutdown();
  return g_passed ? 0 : 1;
}
