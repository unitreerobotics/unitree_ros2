#pragma once

#include "client/base_client.hpp"
#include "client/motion_switch_api.hpp"

namespace unitree
{
namespace ros2
{

class MotionSwitchClient: public BaseClient
{
public:
    MotionSwitchClient(const std::string &nodeName = "motion_switch_client");

    int32_t CheckMode(std::string &form, std::string &name);

    int32_t SelectMode(const std::string &nameOrAlias);

    int32_t ReleaseMode();

    int32_t SetSilent(bool silent);

    int32_t GetSilent(bool &silent);

private:
    MotionSwitchApi mParam;
};

}
}
