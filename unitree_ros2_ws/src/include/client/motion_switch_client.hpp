#pragma once

#include <rclcpp/rclcpp.hpp>
#include "client/motion_switch_api.hpp"
#include "client/config.hpp"

namespace unitree
{
namespace ros2
{

class MotionSwitchClient: public rclcpp::Node,public ClientConfig
{
public:
    MotionSwitchClient(const std::string &nodeName = "motion_switch_client");

    int32_t CheckMode(std::string &form, std::string &name);

    int32_t SelectMode(const std::string &nameOrAlias);

    int32_t ReleaseMode();

    int32_t SetSilent(bool silent);

    int32_t GetSilent(bool &silent);

private:
    bool wait_service();

    MotionSwitchApi mParam;
    rclcpp::Client<unitree_api::srv::Generic>::SharedPtr mClient;
};

}
}
