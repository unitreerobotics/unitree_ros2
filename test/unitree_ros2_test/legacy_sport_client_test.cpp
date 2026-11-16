// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <type_traits>
#ifdef UNITREE_B2_SPORT
#include "common/ros2_b2_sport_client.h"
#else
#include "common/ros2_sport_client.h"
#endif

static_assert(std::is_copy_constructible<SportClient>::value,
              "The legacy SportClient remains copy constructible");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "correlation";
  auto node = std::make_shared<rclcpp::Node>("legacy_sport_probe");
  auto peer = std::make_shared<rclcpp::Node>("legacy_sport_peer");
  auto publisher = peer->create_publisher<unitree_api::msg::Response>("/api/sport/response", 10);
  auto subscription = peer->create_subscription<unitree_api::msg::Request>(
    "/api/sport/request", 10, [publisher, mode](unitree_api::msg::Request::ConstSharedPtr req) {
      if (mode == "timeout") {return;}
      unitree_api::msg::Response response;
      const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (!publisher->get_subscription_count() && std::chrono::steady_clock::now() < discovery_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      response.header.identity = req->header.identity;
      if (mode == "status") {
        response.header.status.code = -23;
      } else {
        response.header.identity.id++;
        response.data = R"({"data":false})";
        publisher->publish(response);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        response.header.identity = req->header.identity;
      }
      response.data = R"({"data":true})";
      publisher->publish(response);
      publisher->publish(response);
    });
  if (mode == "no-server") {
    subscription.reset();
    publisher.reset();
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.add_node(peer);
  std::atomic<bool> spin_failed{false};
  std::thread spin([&]() {
    try {executor.spin();}
    catch (const std::exception &error) {std::cerr << error.what() << std::endl; spin_failed = true;}
  });
  bool passed = false;
  {
    SportClient client(node.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    unitree_api::msg::Request req;
    req.header.identity.api_id = 2055;
    const auto start = std::chrono::steady_clock::now();
    try {
      const auto result = client.Call<unitree_api::msg::Request, unitree_api::msg::Response>(req);
      passed = mode == "correlation" && result.at("data").get<bool>();
    } catch (const std::runtime_error &error) {
      const auto elapsed = std::chrono::steady_clock::now() - start;
      const int32_t expected = mode == "status" ? -23 :
        mode == "no-server" ? UT_ROBOT_TASK_NOT_PUBLISHED : UT_ROBOT_TASK_TIMEOUT;
      const bool correct_error = std::string(error.what()) ==
        "Unitree sport API call failed: " + std::to_string(expected);
      passed = correct_error && (mode == "status" ||
        ((mode == "timeout" || mode == "no-server") &&
        elapsed >= std::chrono::seconds(4) && elapsed < std::chrono::seconds(7)));
      std::cout << "caught: " << error.what() << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return passed && !spin_failed ? 0 : 1;
}
