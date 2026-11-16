// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// A server that reads requests but has no response writer: the call must time
// out without publishing, because a caller told "timeout" must be able to rely
// on the server never acting on that request. Built for the generic client and,
// with UNITREE_B2_CLIENT, for the B2 client.
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#ifdef UNITREE_B2_CLIENT
#include "common/b2_base_client.hpp"
#else
#include "common/base_client.hpp"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string request_topic = "/unitree_test/unmatched/request";
  auto node = std::make_shared<rclcpp::Node>("unitree_unmatched_client");
  auto server = std::make_shared<rclcpp::Node>("unitree_unmatched_server");
  std::atomic<int> received{0};
  auto subscription = server->create_subscription<unitree_api::msg::Request>(
    request_topic, 10,
    [&received](unitree_api::msg::Request::ConstSharedPtr) {received.fetch_add(1);});
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(server);
#ifndef UNITREE_B2_CLIENT
  executor.add_node(node);  // The generic client relies on the caller spinning its node.
#endif
  std::thread spin([&executor]() {executor.spin();});
  int32_t result = 0;
  size_t request_readers = 0;
  {
    BaseClient client(node.get(), request_topic, "/unitree_test/unmatched/response");
    client.SetTimeout(std::chrono::milliseconds(500));
    unitree_api::msg::Request request;
    request.header.identity.api_id = 1001;
    nlohmann::json js;
    result = client.Call(request, js);
    // The request side is matched, so a published request would arrive.
    request_readers = node->count_subscribers(request_topic);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  executor.cancel();
  spin.join();
  std::cout << "result=" << result << " request_readers=" << request_readers <<
    " requests_received=" << received.load() << std::endl;
  rclcpp::shutdown();
  return result == UT_ROBOT_TASK_NOT_PUBLISHED && request_readers > 0 && received.load() == 0 ? 0 : 1;
}
