/**
 * @file g1_arm_action_example.cpp
 * @brief This example demonstrates how to use the G1 Arm Action Client to
 * execute predefined arm actions in ROS2.
 */
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "g1/g1_arm_action_client.hpp"
#include "rclcpp/rclcpp.hpp"

class G1ArmActionNode : public rclcpp::Node {
 public:
  explicit G1ArmActionNode() : Node("g1_arm_action_node") {
    client_ = std::make_shared<unitree::robot::g1::G1ArmActionClient>(this);

    thread_ = std::jthread([this] {
      using namespace std::chrono_literals;
      while (true) {
        std::this_thread::sleep_for(100ms);
        checkForInput();

      }
    });
  }

 private:
  void checkForInput() {
    static std::string line;
    static bool getting_input = false;
    RCLCPP_INFO(this->get_logger(), "G1 Arm Action Node initialized");
    RCLCPP_INFO(this->get_logger(), "Usage:");
    RCLCPP_INFO(this->get_logger(), "  - 0: print supported actions.");
    RCLCPP_INFO(this->get_logger(), "  - an id: execute an action.");
    RCLCPP_INFO(this->get_logger(), "Attention:");
    RCLCPP_INFO(this->get_logger(),
                "  Some actions will not be displayed on the APP,");
    RCLCPP_INFO(this->get_logger(), "  but can be executed by the program.");
    RCLCPP_INFO(this->get_logger(),
                "  These actions may cause the robot to fall,");
    RCLCPP_INFO(this->get_logger(), "  so please execute them with caution.");

    if (!getting_input) {
      std::cout << "\nEnter action ID (or 'q' to quit): ";
      getting_input = true;
    }

    if (std::cin.peek() != EOF) {
      std::getline(std::cin, line);
      getting_input = false;

      if (line == "q") {
        rclcpp::shutdown();
        return;
      }

      try {
        int32_t action_id = std::stoi(line);
        ProcessAction(action_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid input: %s. Please enter an integer.", e.what());
      }
    }
  }

  void ProcessAction(int32_t action_id) {
    if (action_id == 0) {
      std::string action_list_data;
      int32_t ret = client_->GetActionList(action_list_data);
      if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to get action list, error code: %d", ret);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Available actions:\n%s",
                  action_list_data.c_str());
    } else {
      int32_t ret = client_->ExecuteAction(action_id);
      if (ret != 0) {
        handleActionError(ret);
      } else {
        RCLCPP_INFO(this->get_logger(), "Action %d executed successfully",
                    action_id);
      }
    }
  }

  void handleActionError(int32_t error_code) {
    switch (error_code) {
      case unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
        RCLCPP_ERROR(this->get_logger(), "%s",
                     unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC);
        break;
      case unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_HOLDING:
        RCLCPP_ERROR(this->get_logger(), "%s",
                     unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC);
        break;
      case unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
        RCLCPP_ERROR(
            this->get_logger(), "%s",
            unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC);
        break;
      case unitree::robot::g1::UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
        RCLCPP_ERROR(
            this->get_logger(),
            "The actions are only supported in fsm id {500, 501, 801}");
        RCLCPP_ERROR(this->get_logger(),
                     "You can subscribe the topic rt/sportmodestate to check "
                     "the fsm id.");
        RCLCPP_ERROR(this->get_logger(),
                     "And in the state 801, the actions are only supported in "
                     "the fsm mode {0, 3}.");
        RCLCPP_ERROR(
            this->get_logger(),
            "If an error is still returned at this point, ignore this action.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(),
                     "Execute action failed, error code: %d", error_code);
        break;
    }
  }

  std::jthread thread_;
  std::shared_ptr<unitree::robot::g1::G1ArmActionClient> client_;
};

int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<G1ArmActionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}