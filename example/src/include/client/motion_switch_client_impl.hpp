#pragma once

#include <rclcpp/rclcpp.hpp>
#include "motion_switch_client_parameter.hpp"

namespace unitree
{
namespace ros2
{
namespace g1
{

class MotionSwitchClient: public rclcpp::Node
{
public:
    MotionSwitchClient();

    int32_t CheckMode(std::string &form, std::string &name);

    int32_t SelectMode(const std::string &nameOrAlias);

    int32_t ReleaseMode();

    int32_t SetSilent(bool silent);

    int32_t GetSilent(bool &silent);

private:
    bool wait_service();

    MotionSwitchClientParameter mParam;
    rclcpp::Client<unitree_api::srv::Generic>::SharedPtr mClient;
};

}
}
}