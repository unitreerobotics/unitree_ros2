// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#ifdef UNITREE_B2_CLIENT
#include "common/b2_base_client.hpp"
#else
#include "common/base_client.hpp"
#endif
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const bool matched = argc > 1 && std::string(argv[1]) == "expired";
  auto node = std::make_shared<rclcpp::Node>("deadline_client");
  auto peer = std::make_shared<rclcpp::Node>("deadline_peer");
  std::atomic<int> requests{0};
  auto reader = peer->create_subscription<unitree_api::msg::Request>(
    "/unitree_test/deadline/request", 10, [&](unitree_api::msg::Request::ConstSharedPtr) {++requests;});
  rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr writer;
  if (matched) writer = peer->create_publisher<unitree_api::msg::Response>(
    "/unitree_test/deadline/response", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  std::thread spin([&] {executor.spin();});
  int rc;
  {
    BaseClient client(node.get(), "/unitree_test/deadline/request",
      "/unitree_test/deadline/response");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    client.SetTimeout(matched ? std::chrono::milliseconds(0) : std::chrono::milliseconds(100));
    unitree_api::msg::Request request;
    request.header.identity.api_id = 1001;
    const auto start = std::chrono::steady_clock::now();
    rc = client.Call(request);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "result=" << rc << " requests=" << requests.load()
      << " elapsed_ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << std::endl;
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return rc == UT_ROBOT_TASK_NOT_PUBLISHED && requests == 0 ? 0 : 1;
}
