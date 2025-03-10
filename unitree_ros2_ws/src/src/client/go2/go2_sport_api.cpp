#include "client/go2/go2_sport_api.hpp"
#include "nlohmann/json.hpp"

using namespace unitree::ros2::go2;

void SportClientApi::DampReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->priority = true;
    req->api_id = ROBOT_SPORT_API_ID_DAMP;
}

int32_t SportClientApi::DampRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::BalanceStandReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
}

int32_t SportClientApi::BalanceStandRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::StopMoveReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->priority = true;
    req->api_id = ROBOT_SPORT_API_ID_STOPMOVE;
}

int32_t SportClientApi::StopMoveRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::StandUpReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_STANDUP;
}

int32_t SportClientApi::StandUpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::StandDownReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_STANDDOWN;
}

int32_t SportClientApi::StandDownRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::RecoveryStandReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
}

int32_t SportClientApi::RecoveryStandRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::EulerReq(float roll, float pitch, float yaw, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["x"] = roll;
    js["y"] = pitch;
    js["z"] = yaw;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_EULER;
}

int32_t SportClientApi::EulerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::MoveReq(float vx, float vy, float vyaw, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_MOVE;
    req->fastreply = true;
}

int32_t SportClientApi::MoveRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::SitReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_SIT;
}

int32_t SportClientApi::SitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::RiseSitReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_RISESIT;
}

int32_t SportClientApi::RiseSitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::SwitchGaitReq(int32_t d, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = d;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
}

int32_t SportClientApi::SwitchGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::TriggerReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_TRIGGER;
}

int32_t SportClientApi::TriggerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::BodyHeightReq(float height, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = height;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
}

int32_t SportClientApi::BodyHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FootRaiseHeightReq(float height, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = height;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT;
}

int32_t SportClientApi::FootRaiseHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::SpeedLevelReq(int32_t level, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = level;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
}

int32_t SportClientApi::SpeedLevelRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::HelloReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_HELLO;
}

int32_t SportClientApi::HelloRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::StretchReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_STRETCH;
}

int32_t SportClientApi::StretchRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::TrajectoryFollowReq(const std::vector<PathPoint> &path, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json json_array = nlohmann::json::array();
    for (const auto &item : path)
    {
        nlohmann::json obj;
        obj["t_from_start"] = item.timeFromStart;
        obj["x"] = item.x;
        obj["y"] = item.y;
        obj["yaw"] = item.yaw;
        obj["vx"] = item.vx;
        obj["vy"] = item.vy;
        obj["vyaw"] = item.vyaw;
        json_array.emplace_back(obj);
    }
    req->parameter = json_array.dump();
    req->api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
    req->fastreply = true;
}

int32_t SportClientApi::TrajectoryFollowRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::SwitchJoystickReq(bool on, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = on;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
}

int32_t SportClientApi::SwitchJoystickRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::ContinuousGaitReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT;
}

int32_t SportClientApi::ContinuousGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::WallowReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_WALLOW;
}

int32_t SportClientApi::WallowRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::ContentReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_CONTENT;
}

int32_t SportClientApi::ContentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::HeartReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_HEART;
}

int32_t SportClientApi::HeartRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::PoseReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_POSE;
}

int32_t SportClientApi::PoseRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::ScrapeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_SCRAPE;
}

int32_t SportClientApi::ScrapeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FrontFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
}

int32_t SportClientApi::FrontFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FrontJumpReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
}

int32_t SportClientApi::FrontJumpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FrontPounceReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
}

int32_t SportClientApi::FrontPounceRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::Dance1Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_DANCE1;
}

int32_t SportClientApi::Dance1Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::Dance2Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_DANCE2;
}

int32_t SportClientApi::Dance2Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::Dance3Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_DANCE3;
}

int32_t SportClientApi::Dance3Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::Dance4Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_DANCE4;
}

int32_t SportClientApi::Dance4Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::HopSpinLeftReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_HOPSPINLEFT;
}

int32_t SportClientApi::HopSpinLeftRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::HopSpinRightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_HOPSPINRIGHT;
}

int32_t SportClientApi::HopSpinRightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::WiggleHipsReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_WIGGLEHIPS;
}

int32_t SportClientApi::WiggleHipsRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::GetStateReq(const std::vector<std::string> &status, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js = status;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_GETSTATE;
}

int32_t SportClientApi::GetStateRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, std::map<std::string, std::string> &statusMap)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        statusMap = js.get<std::map<std::string, std::string>>();
    }
    return res->code;
}

void SportClientApi::EconomicGaitReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_GETSTATE;
}

int32_t SportClientApi::EconomicGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::LeftFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_LEFTFLIP;
}

int32_t SportClientApi::LeftFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::BackFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_SPORT_API_ID_BACKFLIP;
}

int32_t SportClientApi::BackFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FreeWalkReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = true;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_FREEWALK;
}

int32_t SportClientApi::FreeWalkRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FreeBoundReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_FREEBOUND;
}

int32_t SportClientApi::FreeBoundRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FreeJumpReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_FREEJUMP;
}

int32_t SportClientApi::FreeJumpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::FreeAvoidReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_FREEAVOID;
}

int32_t SportClientApi::FreeAvoidRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::WalkStairReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_WALKSTAIR;
}

int32_t SportClientApi::WalkStairRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::WalkUprightReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_WALKUPRIGHT;
}

int32_t SportClientApi::WalkUprightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void SportClientApi::CrossStepReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["data"] = flag;
    req->parameter = js.dump();
    req->api_id = ROBOT_SPORT_API_ID_CROSSSTEP;
}

int32_t SportClientApi::CrossStepRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}
