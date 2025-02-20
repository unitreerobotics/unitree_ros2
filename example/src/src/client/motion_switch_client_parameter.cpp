#include "motion_switch_client_parameter.hpp"
#include "nlohmann/json.hpp"

const int32_t MOTION_SWITCHER_API_ID_CHECK_MODE = 1001;
const int32_t MOTION_SWITCHER_API_ID_SELECT_MODE = 1002;
const int32_t MOTION_SWITCHER_API_ID_RELEASE_MODE = 1003;
const int32_t MOTION_SWITCHER_API_ID_SET_SILENT = 1004;
const int32_t MOTION_SWITCHER_API_ID_GET_SILENT = 1005;

using namespace unitree::ros2::g1;

void MotionSwitchClientParameter::CheckModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_CHECK_MODE;
}

int32_t MotionSwitchClientParameter::CheckModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, std::string &form, std::string &name)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        form = js["form"];
        name = js["name"];
    }
    return res->code;
}

void MotionSwitchClientParameter::SelectModeReq(const std::string &nameOrAlias, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["name"] = nameOrAlias;
    req->parameter = js.dump();
    req->api_id = MOTION_SWITCHER_API_ID_SELECT_MODE;
}

int32_t unitree::ros2::g1::MotionSwitchClientParameter::SelectModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchClientParameter::ReleaseModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_RELEASE_MODE;
}

int32_t MotionSwitchClientParameter::ReleaseModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchClientParameter::SetSilentReq(bool silent, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["silent"] = silent;
    req->parameter = js.dump();
    req->api_id = MOTION_SWITCHER_API_ID_SET_SILENT;
}

int32_t MotionSwitchClientParameter::SetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void MotionSwitchClientParameter::GetSilentReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = MOTION_SWITCHER_API_ID_GET_SILENT;
}

int32_t MotionSwitchClientParameter::GetSilentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, bool &silent)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        silent = js["silent"];
    }
    return res->code;
}
