// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Exercise the existing client against a local mock on private topics.
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include "common/base_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "success";
  if (mode == "unique-ids") {
    // Every client on a response topic sees every response and correlates it by
    // (identity.id, api_id), so request ids must never repeat within a process.
    std::vector<std::vector<int64_t>> ids(4);
    std::vector<std::thread> threads;
    for (auto & thread_ids : ids) {
      threads.emplace_back([&thread_ids]() {
          for (int i = 0; i < 20000; ++i) {
            thread_ids.push_back(unitree::common::NextRequestId());
          }
        });
    }
    for (auto & thread : threads) {thread.join();}
    std::set<int64_t> unique;
    bool increasing = true;
    for (const auto & thread_ids : ids) {
      for (size_t i = 1; i < thread_ids.size(); ++i) {
        increasing = increasing && thread_ids[i] > thread_ids[i - 1];
      }
      unique.insert(thread_ids.begin(), thread_ids.end());
    }
    std::cout << "unique-ids: " << unique.size() << " unique, increasing=" << increasing << std::endl;
    rclcpp::shutdown();
    return unique.size() == 80000 && increasing ? 0 : 1;
  }
  auto node = std::make_shared<rclcpp::Node>("unitree_api_client_test");
  auto server = std::make_shared<rclcpp::Node>("unitree_api_mock");
  auto publisher = server->create_publisher<unitree_api::msg::Response>(
    "/unitree_test/client/response", 10);
  std::atomic<bool> request_valid{false};
  auto subscription = server->create_subscription<unitree_api::msg::Request>(
    "/unitree_test/client/request", 10,
    [publisher, mode, &request_valid](unitree_api::msg::Request::ConstSharedPtr request) {
      request_valid.store(
        request->header.identity.id > 0 && request->header.identity.api_id == 1001 &&
        request->header.lease.id == 123456789 && request->header.policy.priority == 7 &&
        !request->header.policy.noreply && request->parameter == "{\"probe\":true}" &&
        request->binary == std::vector<uint8_t>({0, 127, 255}));
      if (mode == "timeout") {return;}
      unitree_api::msg::Response response;
      response.header.identity = request->header.identity;
      if (mode == "success") {
        response.header.identity.id++;
        response.data = "{\"probe\":\"wrong-id\"}";
        publisher->publish(response);
        response.header.identity = request->header.identity;
        response.header.identity.api_id++;
        response.data = "{\"probe\":\"wrong-api\"}";
        publisher->publish(response);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        response.header.identity = request->header.identity;
      }
      response.header.status.code = mode == "status" ? -23 : 0;
      response.data = "{\"probe\":\"ok\"}";
      if (mode == "malformed") {response.data = "{invalid-json";}
      if (mode == "empty-json" || mode == "status-only") {response.data.clear();}
      if (mode == "escaped-null") {response.data = R"({"probe":"\u0000"})";}
      if (mode == "embedded-null") {
        response.data.push_back('\0');
        response.data += "invalid-trailing-data";
      }
      publisher->publish(response);
      publisher->publish(response);    // Duplicate delivery must be harmless.
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.add_node(server);
  std::atomic<bool> spin_failed{false};
  std::thread spin([&]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        std::cerr << error.what() << std::endl;
        spin_failed.store(true);
      }
    });
  BaseClient client(node.get(), "/unitree_test/client/request",
    "/unitree_test/client/response");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  unitree_api::msg::Request request;
  request.header.identity.api_id = 1001;
  request.header.lease.id = 123456789;
  request.header.policy.priority = 7;
  request.header.policy.noreply = false;
  request.parameter = "{\"probe\":true}";
  request.binary = {0, 127, 255};
  nlohmann::json data = {{"stale", true}};
  const int result = mode == "status-only" ? client.Call(request) :
    client.Call(request, data);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.cancel();
  spin.join();
  const int expected = mode == "status" ? -23 :
    mode == "timeout" ? UT_ROBOT_TASK_TIMEOUT :
    (mode == "malformed" || mode == "empty-json" || mode == "embedded-null") ?
    UT_ROBOT_TASK_UNKNOWN_ERROR : UT_ROBOT_SUCCESS;
  bool passed = result == expected && !spin_failed.load() && request_valid.load();
  if (mode == "success") {passed = passed && data.value("probe", "") == "ok";}
  if (mode == "escaped-null") {
    passed = passed && data.at("probe").get<std::string>() == std::string(1, '\0');
  }
  if (expected != UT_ROBOT_SUCCESS || mode == "status-only") {
    passed = passed && data == nlohmann::json({{"stale", true}});
  }
  std::cout << mode << ": result=" << result << " data=" << data.dump() << std::endl;
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
