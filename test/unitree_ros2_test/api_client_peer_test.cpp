// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Drive an API client against the separate plain DDS server in dds_api_server.c.
// Built twice: the generic client (common/base_client.hpp) and, with
// UNITREE_B2_CLIENT, the B2 client (common/b2_base_client.hpp).
//   scenario   first call with no settle time, decoys, duplicates and a late
//              response during the next call (both clients serialize calls;
//              see api_single_flight_test.cpp)
//   reconnect  the server exits and a new one starts under the same client
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#ifdef UNITREE_B2_CLIENT
#include "common/b2_base_client.hpp"
#else
#include "common/base_client.hpp"
#endif

namespace
{
bool g_passed = true;

void expect(bool condition, const std::string & what)
{
  std::cout << (condition ? "ok   " : "FAIL ") << what << std::endl;
  g_passed = g_passed && condition;
}

bool genuine(const nlohmann::json & js, int64_t api)
{
  return js.is_object() && js.value("api", int64_t{0}) == api && !js.value("decoy", true);
}

int32_t call(BaseClient & client, int64_t api, nlohmann::json & js)
{
  unitree_api::msg::Request request;
  request.header.identity.api_id = api;
  return client.Call(request, js);
}

void scenario(BaseClient & client)
{
  // 1. No settle time: the request and response endpoints are still being
  //    discovered. Decoys with a wrong id or api and a duplicate follow.
  nlohmann::json first;
  expect(call(client, 2001, first) == UT_ROBOT_SUCCESS, "first call right after construction");
  expect(genuine(first, 2001), "first call ignores decoys: " + first.dump());

  // 2. A response delivered after its call timed out must not complete or
  //    corrupt the next call, which is pending when the late response arrives.
  nlohmann::json late = {{"untouched", true}};
  client.SetTimeout(std::chrono::milliseconds(300));
  expect(call(client, 2004, late) == UT_ROBOT_TASK_TIMEOUT, "short timeout expires");
  expect(late == nlohmann::json({{"untouched", true}}), "timed-out call leaves output");
  client.SetTimeout(std::chrono::seconds(5));
  nlohmann::json next;
  expect(call(client, 2005, next) == UT_ROBOT_SUCCESS && genuine(next, 2005),
    "next call ignores the late response and a cross-api decoy: " + next.dump());
}

void reconnect(BaseClient & client)
{
  // First server answers one call and exits; the test harness starts a second
  // server about a second later. Calls in the gap time out; the client must
  // then reach the new server without being recreated.
  nlohmann::json first;
  expect(call(client, 2001, first) == UT_ROBOT_SUCCESS && genuine(first, 2001),
    "call to the first server: " + first.dump());
  // Long enough for a request/response round trip on a slow link.
  client.SetTimeout(std::chrono::milliseconds(1500));
  int timeouts = 0;
  bool reconnected = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!reconnected && std::chrono::steady_clock::now() < deadline) {
    nlohmann::json js;
    const int32_t result = call(client, 2001, js);
    if (result == UT_ROBOT_SUCCESS) {
      reconnected = genuine(js, 2001);
    } else if (result == UT_ROBOT_TASK_NOT_PUBLISHED || result == UT_ROBOT_TASK_TIMEOUT) {
      ++timeouts;
    } else {
      expect(false, "unexpected result " + std::to_string(result));
      return;
    }
  }
  expect(timeouts > 0, "calls time out while no server runs (" + std::to_string(timeouts) + ")");
  expect(reconnected, "same client reaches the restarted server");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "scenario";
  const char * ns = std::getenv("UNITREE_DDS_TEST_NAMESPACE");
  if (ns == nullptr || (mode != "scenario" && mode != "reconnect")) {
    std::cerr << "usage: UNITREE_DDS_TEST_NAMESPACE=... " << argv[0] <<
      " [scenario|reconnect]" << std::endl;
    return 2;
  }
  const std::string prefix = std::string("/") + ns;
  auto node = std::make_shared<rclcpp::Node>("unitree_api_peer_client");
#ifndef UNITREE_B2_CLIENT
  // The generic client relies on the caller spinning its node; the B2 client
  // spins its own response callback group.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::atomic<bool> spin_failed{false};
  std::thread spin([&]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        std::cerr << error.what() << std::endl;
        spin_failed.store(true);
      }
    });
#endif
  {
    BaseClient client(node.get(), prefix + "/request", prefix + "/response");
    if (mode == "scenario") {
      scenario(client);
    } else {
      reconnect(client);
    }
  }
#ifndef UNITREE_B2_CLIENT
  executor.cancel();
  spin.join();
  expect(!spin_failed.load(), "executor did not throw");
#endif
  rclcpp::shutdown();
  return g_passed ? 0 : 1;
}
