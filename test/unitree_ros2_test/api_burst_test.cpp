// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Retain a complete response burst while the caller's executor is occupied.
// Every client of an API sees every response on the shared response topic.
// Eight clients each have one call in flight; the peer answers all eight in
// reverse order, twice, so each client's reader receives sixteen samples
// before its executor resumes.
#include <atomic>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "common/base_client.hpp"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("unitree_api_burst_client");
  auto server = std::make_shared<rclcpp::Node>("unitree_api_burst_peer");
  auto publisher = server->create_publisher<unitree_api::msg::Response>(
    "/unitree_test/burst/response", 100);
  std::vector<unitree_api::msg::Response> responses;
  std::atomic<bool> sent{false};
  auto subscription = server->create_subscription<unitree_api::msg::Request>(
    "/unitree_test/burst/request", 100,
    [&](unitree_api::msg::Request::ConstSharedPtr request) {
      unitree_api::msg::Response response;
      response.header.identity = request->header.identity;
      response.data = "{\"api\":" + std::to_string(request->header.identity.api_id) + "}";
      responses.push_back(response);
      if (responses.size() == 8) {
        for (auto it = responses.rbegin(); it != responses.rend(); ++it) {
          publisher->publish(*it);
          publisher->publish(*it);
        }
        sent.store(true);
      }
    });
  rclcpp::executors::SingleThreadedExecutor server_executor;
  server_executor.add_node(server);
  std::thread server_spin([&]() {server_executor.spin();});
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(node);
  std::atomic<int> passed{0};
  {
    std::vector<std::unique_ptr<BaseClient>> clients;
    for (int api = 1; api <= 8; ++api) {
      clients.push_back(std::make_unique<BaseClient>(node.get(), "/unitree_test/burst/request",
        "/unitree_test/burst/response"));
      clients.back()->SetTimeout(2s);
    }
    std::vector<std::thread> calls;
    for (int api = 1; api <= 8; ++api) {
      calls.emplace_back([&, api]() {
        unitree_api::msg::Request request;
        request.header.identity.api_id = api;
        nlohmann::json data;
        if (clients[api - 1]->Call(request, data) == UT_ROBOT_SUCCESS &&
          data.value("api", 0) == api)
        {
          passed.fetch_add(1);
        }
      });
    }
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!sent.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    // DDS receives samples independently of the ROS executor. All sixteen
    // samples must fit until the client executor resumes taking responses.
    std::this_thread::sleep_for(200ms);
    std::thread client_spin([&]() {client_executor.spin();});
    for (auto & call : calls) {call.join();}
    client_executor.cancel();
    client_spin.join();
  }
  server_executor.cancel();
  server_spin.join();
  rclcpp::shutdown();
  std::cout << "burst_sent=" << sent.load() << " completed=" << passed.load()
            << "/8" << std::endl;
  return sent.load() && passed.load() == 8 ? 0 : 1;
}
