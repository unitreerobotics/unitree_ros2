#include "client/g1/g1_loco_api.hpp"
#include "nlohmann/json.hpp"

using namespace unitree::ros2::g1;

void LocoClientApi::GetFsmIdReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_FSM_ID;
}

int32_t LocoClientApi::GetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &fsm_id)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        fsm_id = js.at("data");
    }
    return res->code;
}

void LocoClientApi::GetFsmModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_FSM_MODE;
}

int32_t LocoClientApi::GetFsmModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &fsm_mode)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        fsm_mode = js.at("data");
    }
    return res->code;
}

void LocoClientApi::GetBalanceModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_BALANCE_MODE;
}

int32_t LocoClientApi::GetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &balance_mode)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        balance_mode = js.at("data");
    }
    return res->code;
}

void LocoClientApi::GetSwingHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_SWING_HEIGHT;
}

int32_t LocoClientApi::GetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &swing_height)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        swing_height = js.at("data");
    }
    return res->code;
}

void LocoClientApi::GetStandHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_STAND_HEIGHT;
}

int32_t LocoClientApi::GetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &stand_height)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        stand_height = js.at("data");
    }
    return res->code;
}

void LocoClientApi::GetPhaseReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_PHASE;
}

int32_t LocoClientApi::GetPhaseRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res,std::vector<float>& phase)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        phase = js["data"].get<std::vector<float>>();
    }
    return res->code;
}

void LocoClientApi::SetFsmIdReq(int32_t fsmId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = fsmId;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
}

int32_t LocoClientApi::SetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientApi::SetBalanceModeReq(int32_t balanceMode, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = balanceMode;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_BALANCE_MODE;
}

int32_t LocoClientApi::SetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientApi::SetSwingHeightReq(float swingHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = swingHeight;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_SWING_HEIGHT;
}

int32_t LocoClientApi::SetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientApi::SetStandHeightReq(float standHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = standHeight;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_STAND_HEIGHT;
}

int32_t LocoClientApi::SetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientApi::SetVelocityReq(float vx, float vy, float omega, float duration, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    std::vector<float> velocity = {vx, vy, omega};
    js["velocity"] = velocity;
    js["duration"] = duration;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
}

int32_t LocoClientApi::SetVelocityRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientApi::SetTaskIdReq(int32_t taskId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = taskId;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_ARM_TASK;
}

int32_t LocoClientApi::SetTaskIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}
