#include "ros2_sport_client.h"

void SportClient::Damp(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
}

void SportClient::BalanceStand(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
}

void SportClient::StopMove(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
}

void SportClient::StandUp(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
}

void SportClient::StandDown(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
}

void SportClient::RecoveryStand(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
}

void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw)
{
    nlohmann::json js;
    js["x"] = roll;
    js["y"] = pitch;
    js["z"] = yaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
}

void SportClient::Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw)
{
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
}

void SportClient::Sit(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT;
}

void SportClient::RiseSit(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT;
}

void SportClient::SwitchGait(unitree_api::msg::Request &req, int d)
{
    nlohmann::json js;
    js["data"] = d;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
    req.parameter = js.dump();
}

void SportClient::Trigger(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_TRIGGER;
}

void SportClient::BodyHeight(unitree_api::msg::Request &req, float height)
{
    nlohmann::json js;
    js["data"] = height;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
}

void SportClient::FootRaiseHeight(unitree_api::msg::Request &req, float height)
{
    nlohmann::json js;
    js["data"] = height;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT;
}

void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level)
{
    nlohmann::json js;
    js["data"] = level;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
}

void SportClient::Hello(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO;
}

void SportClient::Stretch(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH;
}


void SportClient::TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path)
{
    nlohmann::json js_path;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
    for (int i = 0; i < 30; i++)
    {
        nlohmann::json js_point;
        js_point["t_from_start"] = path[i].timeFromStart;
        js_point["x"] = path[i].x;
        js_point["y"] = path[i].y;
        js_point["yaw"] = path[i].yaw;
        js_point["vx"] = path[i].vx;
        js_point["vy"] = path[i].vy;
        js_point["vyaw"] = path[i].vyaw;
        js_path.push_back(js_point);
    }
    req.parameter =js_path.dump();
}

void SportClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
}

void SportClient::ContinuousGait(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT;
}

void SportClient::Wallow(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_WALLOW;
}

void SportClient::Content(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT;
}

void SportClient::Pose(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE;
}

void SportClient::Scrape(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE;
}

void SportClient::FrontFlip(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
}

void SportClient::FrontJump(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
}

void SportClient::FrontPounce(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
}

void SportClient::Dance1(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1;
}

void SportClient::Dance2(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2;
}