#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_b2_sport_client.h"

using namespace std::chrono_literals;

struct TestOption
{
    std::string name;
    int id;
};

const std::vector<TestOption> option_list = {
    {"damp", 0},
    {"balance_stand", 1},
    {"stop_move", 2},
    {"stand_down", 3},
    {"recovery_stand", 4},
    {"move", 5},
    {"switch_gait", 6},
    {"speed_level", 7},
    {"hand_stand", 8},
    {"auto_recovery_set", 9},
    {"free_walk", 10},
    {"classic_walk", 11},
    {"fast_walk", 12},
    {"euler", 13},
};

int ConvertToInt(const std::string &str)
{
    try
    {
        return std::stoi(str);
    }
    catch (const std::invalid_argument &)
    {
        return -1;
    }
    catch (const std::out_of_range &)
    {
        return -1;
    }
}

class UserInterface
{
public:
    UserInterface() = default;

    void terminalHandle()
    {
        std::string input;
        std::getline(std::cin, input);

        if (input == "list")
        {
            for (const auto &option : option_list)
            {
                std::cout << option.name << ", id: " << option.id << std::endl;
            }
            return;
        }

        for (const auto &option : option_list)
        {
            if (input == option.name || ConvertToInt(input) == option.id)
            {
                test_option_->id = option.id;
                test_option_->name = option.name;
                std::cout << "Test: " << test_option_->name << ", test_id: " << test_option_->id << std::endl;
            }
        }
    }

    TestOption *test_option_{nullptr};
};

class B2wSportClientNode : public rclcpp::Node
{
public:
    B2wSportClientNode() : Node("b2w_sport_client_node"), sport_client_(this)
    {
        test_option_.id = 1;
        user_interface_.test_option_ = &test_option_;

        t1_ = std::thread([this] {
            // 等待一段时间，让 ROS2 spin 处理完初始启动
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            run();
        });
    }

    void run()
    {
        std::cout << "Input \"list\" to list all test option ..." << std::endl;

        while (1)
        {
            auto time_start_trick = std::chrono::high_resolution_clock::now();
            static const constexpr auto dt = std::chrono::microseconds(20000); // 50Hz

            user_interface_.terminalHandle();
            unitree_api::msg::Request req;

            if (test_option_.id == 0)
            {
                sport_client_.Damp(req);
            }
            else if (test_option_.id == 1)
            {
                sport_client_.BalanceStand(req);
            }
            else if (test_option_.id == 2)
            {
                sport_client_.StopMove(req);
            }
            else if (test_option_.id == 3)
            {
                sport_client_.StandDown(req);
            }
            else if (test_option_.id == 4)
            {
                sport_client_.RecoveryStand(req);
            }
            else if (test_option_.id == 5)
            {
                sport_client_.Move(req, 0.0, 0.0, 0.5);
            }
            else if (test_option_.id == 6)
            {
                sport_client_.SwitchGait(req, 0);
            }
            else if (test_option_.id == 7)
            {
                sport_client_.SpeedLevel(req, 1);

            }
            else if (test_option_.id == 8)
            {
                sport_client_.HandStand(req, true);

            }
            else if (test_option_.id == 9)
            {
                sport_client_.AutoRecoverySet(req, true);
            }
            else if (test_option_.id == 10)
            {
                sport_client_.FreeWalk(req);
            }
            else if (test_option_.id == 11)
            {
                sport_client_.ClassicWalk(req, true);
            }
            else if (test_option_.id == 12)
            {
                sport_client_.FastWalk(req, true);
            }
            else if (test_option_.id == 13)
            {
                sport_client_.Euler(req, 0, 0, 0.6);
            }
            std::this_thread::sleep_until(time_start_trick + dt);
        }
    }

private:
    SportClient sport_client_;
    TestOption test_option_;
    UserInterface user_interface_;
    std::thread t1_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B2wSportClientNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}