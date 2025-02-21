#include "client/motion_switch_api.hpp"
#include "nlohmann/json.hpp"

using namespace unitree::ros2;

void MotionSwitchApi::CheckModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_CHECK_MODE;
}

int32_t MotionSwitchApi::CheckModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, std::string &form, std::string &name)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        form = js["form"];
        name = js["name"];
    }
    return res->code;
}

void MotionSwitchApi::SelectModeReq(const std::string &nameOrAlias, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["name"] = nameOrAlias;
    req->parameter = js.dump();
    req->api_id = MOTION_SWITCHER_API_ID_SELECT_MODE;
}

int32_t MotionSwitchApi::SelectModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchApi::ReleaseModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_RELEASE_MODE;
}

int32_t MotionSwitchApi::ReleaseModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchApi::SetSilentReq(bool silent, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["silent"] = silent;
    req->parameter = js.dump();
    req->api_id = MOTION_SWITCHER_API_ID_SET_SILENT;
}

int32_t MotionSwitchApi::SetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchApi::GetSilentReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_GET_SILENT;
}

int32_t MotionSwitchApi::GetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, bool &silent)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        silent = js["silent"];
    }
    return res->code;
}
