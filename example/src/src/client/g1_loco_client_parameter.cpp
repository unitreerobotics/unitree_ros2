#include "g1_loco_client_parameter.hpp"
#include "nlohmann/json.hpp"

using namespace unitree::ros2::g1;

constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 7001;
constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002;
constexpr int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003;
constexpr int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004;
constexpr int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005;
constexpr int32_t ROBOT_API_ID_LOCO_GET_PHASE = 7006; // deprecated

constexpr int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 7101;
constexpr int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102;
constexpr int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103;
constexpr int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104;
constexpr int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;
constexpr int32_t ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106;

void LocoClientParameter::GetFsmIdReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_FSM_ID;
}

int32_t LocoClientParameter::GetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &fsm_id)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        fsm_id = js.at("data");
    }
    return res->code;
}

void LocoClientParameter::GetFsmModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_FSM_MODE;
}

int32_t LocoClientParameter::GetFsmModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &fsm_mode)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        fsm_mode = js.at("data");
    }
    return res->code;
}

void LocoClientParameter::GetBalanceModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_BALANCE_MODE;
}

int32_t LocoClientParameter::GetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &balance_mode)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        balance_mode = js.at("data");
    }
    return res->code;
}

void LocoClientParameter::GetSwingHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_SWING_HEIGHT;
}

int32_t LocoClientParameter::GetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &swing_height)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        swing_height = js.at("data");
    }
    return res->code;
}

void LocoClientParameter::GetStandHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_STAND_HEIGHT;
}

int32_t LocoClientParameter::GetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &stand_height)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        stand_height = js.at("data");
    }
    return res->code;
}

void LocoClientParameter::GetPhaseReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_LOCO_GET_PHASE;
}

int32_t LocoClientParameter::GetPhaseRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res,std::vector<float>& phase)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        phase = js["data"].get<std::vector<float>>();
    }
    return res->code;
}

void LocoClientParameter::SetFsmIdReq(int32_t fsmId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = fsmId;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_FSM_ID;
}

int32_t LocoClientParameter::SetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientParameter::SetBalanceModeReq(int32_t balanceMode, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = balanceMode;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_BALANCE_MODE;
}

int32_t LocoClientParameter::SetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientParameter::SetSwingHeightReq(float swingHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = swingHeight;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_SWING_HEIGHT;
}

int32_t LocoClientParameter::SetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientParameter::SetStandHeightReq(float standHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = standHeight;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_STAND_HEIGHT;
}

int32_t LocoClientParameter::SetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientParameter::SetVelocityReq(float vx, float vy, float omega, float duration, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    std::vector<float> velocity = {vx, vy, omega};
    js["velocity"] = velocity;
    js["duration"] = duration;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
}

int32_t LocoClientParameter::SetVelocityRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void LocoClientParameter::SetTaskIdReq(int32_t taskId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = taskId;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_LOCO_SET_ARM_TASK;
}

int32_t LocoClientParameter::SetTaskIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}
