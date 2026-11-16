// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
#include <atomic>
#include <iostream>
#include <thread>
#ifdef UNITREE_SHAPE_B2
#include "h2/h2_loco_client.hpp"
#include "h2/h2_motion_switch_client.hpp"
#include "b2/b2_motion_switch_client.hpp"
#else
#include "g1/g1_audio_client.hpp"
#include "g1/g1_loco_client.hpp"
#include "g1/g1_motion_switch_client.hpp"
#include "common/ros2_robot_state_client.h"
#endif
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "volume";
  const bool good = argc > 2 && std::string(argv[2]) == "good";
  auto node = std::make_shared<rclcpp::Node>("response_shape_probe");
  auto peer = std::make_shared<rclcpp::Node>("response_shape_peer");
  std::vector<rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr> publishers;
  std::vector<rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr> subscriptions;
  for (const auto &api : {"voice", "sport", "motion_switcher", "robot_state"}) {
    std::string base = std::string("/api/") + api;
    auto pub = peer->create_publisher<unitree_api::msg::Response>(base + "/response", 10);
    publishers.push_back(pub);
    subscriptions.push_back(peer->create_subscription<unitree_api::msg::Request>(
      base + "/request", 10, [pub, mode, good](unitree_api::msg::Request::ConstSharedPtr req) {
        if (mode == "robot-timeout") {return;}
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        unitree_api::msg::Response response;
        response.header.identity = req->header.identity;
        if (mode == "volume") response.data = R"({"volume":{}})";
        if (mode == "fsm") response.data = R"({"data":{}})";
        if (mode == "phase") response.data = R"({"data":[1,{}]})";
        if (mode == "mode" || mode == "b2-mode") response.data = R"({"name":"changed","form":{}})";
        if (mode == "ids") response.data = R"({"ids":[9],"names":[{}]})";
        if (mode == "silent") response.data = R"({"silent":{}})";
        if (mode == "robot-list") response.data = R"([{"name":"changed","status":1,"protect":0},{}])";
        if (mode == "robot-switch") response.data = R"({"name":"changed","status":{}})";
        if (mode == "robot-good") response.data = R"([{"name":"valid","status":1,"protect":0}])";
        if (mode == "robot-status") {response.data = "[]"; response.header.status.code = -23;}
        if (good) {
          if (mode == "volume") response.data = R"({"volume":23})";
          if (mode == "fsm") response.data = R"({"data":23})";
          if (mode == "phase") response.data = R"({"data":[23]})";
          if (mode == "mode" || mode == "b2-mode") response.data = R"({"name":"new_name","form":"new_form"})";
          if (mode == "ids") response.data = R"({"ids":[23],"names":["new"]})";
          if (mode == "silent") response.data = R"({"silent":false})";
          if (mode == "robot-list") response.data = R"([{"name":"valid","status":1,"protect":0}])";
          if (mode == "robot-switch") response.data = R"({"name":"valid","status":23})";
        }
        pub->publish(response);
      }));
  }
  if (mode == "robot-no-peer") {
    subscriptions.clear();
    publishers.clear();
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
  try {
#ifdef UNITREE_SHAPE_B2
    unitree::robot::h2::H2LocoClient loco(node.get());
    unitree::robot::h2::MotionSwitchClient motion(node.get());
    unitree::robot::b2::MotionSwitchClient b2_motion(node.get());
#else
    unitree::robot::g1::LocoClient loco(node.get());
    unitree::robot::g1::MotionSwitchClient motion(node.get());
#endif
    int result = 99;
    bool preserved = false;
    if (mode == "fsm") {
      int value = 71; result = loco.GetFsmId(value); preserved = value == (good ? 23 : 71);
    } else if (mode == "phase") {
      std::vector<float> value{71}; result = loco.GetPhase(value); preserved = value == std::vector<float>{good ? 23.0f : 71.0f};
    } else if (mode == "mode" || mode == "b2-mode") {
      std::string form = "old_form", name = "old_name";
#ifdef UNITREE_SHAPE_B2
      result = mode == "mode" ? motion.CheckMode(form, name) : b2_motion.CheckMode(form, name);
#else
      result = motion.CheckMode(form, name);
#endif
      preserved = form == (good ? "new_form" : "old_form") && name == (good ? "new_name" : "old_name");
    } else if (mode == "silent") {
      bool value = true; result = motion.GetSilent(value); preserved = value == !good;
#ifdef UNITREE_SHAPE_B2
    } else if (mode == "ids") {
      std::vector<int> ids{71}; std::vector<std::string> names{"old"};
      result = loco.GetAvailableFsmIds(ids, names);
      preserved = ids == std::vector<int>{good ? 23 : 71} && names == std::vector<std::string>{good ? "new" : "old"};
#else
    } else if (mode == "volume") {
      auto audio = std::make_shared<unitree::ros2::g1::AudioClient>();
      executor.add_node(audio);
      uint8_t value = 71; result = audio->GetVolume(value); preserved = value == (good ? 23 : 71);
      executor.remove_node(audio);
    } else if (mode == "robot-switch") {
      RobotStateClient robot(node.get());
      int32_t value = 71; result = robot.ServiceSwitch("test", 0, value); preserved = value == (good ? 23 : 71);
    } else {
      RobotStateClient robot(node.get());
      std::vector<ServiceState> value{{"old", 71, 72}};
      result = robot.ServiceList(value);
      preserved = value.size() == 1 && value[0].name == "old" && value[0].status == 71 && value[0].protect == 72;
      if (mode == "robot-good" || good) preserved = value.size() == 1 && value[0].name == "valid" && value[0].status == 1 && value[0].protect == 0;
#endif
    }
    const int expected = good || mode == "robot-good" ? 0 :
      mode == "robot-status" ? -23 : mode == "robot-timeout" ? UT_ROBOT_TASK_TIMEOUT :
      mode == "robot-no-peer" ? UT_ROBOT_TASK_NOT_PUBLISHED : UT_ROBOT_TASK_UNKNOWN_ERROR;
    std::cout << mode << " result=" << result << " preserved=" << preserved << std::endl;
    passed = result == expected && preserved;
  } catch (const std::exception &error) {
    std::cerr << "getter escaped exception: " << error.what() << std::endl;
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return passed && !spin_failed ? 0 : 1;
}
