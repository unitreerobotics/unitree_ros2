#pragma once

#include "unitree_api/srv/generic.hpp"

namespace unitree
{
namespace ros2
{
namespace g1
{

class MotionSwitchClientParameter
{
public:
    void CheckModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t CheckModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, std::string &form, std::string &name);

    void SelectModeReq(const std::string &nameOrAlias, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SelectModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void ReleaseModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t ReleaseModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetSilentReq(bool silent, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void GetSilentReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, bool &silent);
};
}
}
}