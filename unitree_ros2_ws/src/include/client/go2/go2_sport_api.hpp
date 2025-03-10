#pragma once

#include "unitree_api/srv/generic.hpp"
#include <map>

namespace unitree
{
namespace ros2
{
namespace go2
{

constexpr int32_t ROBOT_SPORT_API_ID_DAMP               = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND       = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE           = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDUP            = 1004;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN          = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND      = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_EULER              = 1007;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE               = 1008;
constexpr int32_t ROBOT_SPORT_API_ID_SIT                = 1009;
constexpr int32_t ROBOT_SPORT_API_ID_RISESIT            = 1010;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHGAIT         = 1011;
constexpr int32_t ROBOT_SPORT_API_ID_TRIGGER            = 1012;
constexpr int32_t ROBOT_SPORT_API_ID_BODYHEIGHT         = 1013;
constexpr int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT    = 1014;
constexpr int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL         = 1015;
constexpr int32_t ROBOT_SPORT_API_ID_HELLO              = 1016;
constexpr int32_t ROBOT_SPORT_API_ID_STRETCH            = 1017;
constexpr int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   = 1018;
constexpr int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT     = 1019;
constexpr int32_t ROBOT_SPORT_API_ID_CONTENT            = 1020;
constexpr int32_t ROBOT_SPORT_API_ID_WALLOW             = 1021;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE1             = 1022;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE2             = 1023;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK     = 1027;
constexpr int32_t ROBOT_SPORT_API_ID_POSE               = 1028;
constexpr int32_t ROBOT_SPORT_API_ID_SCRAPE             = 1029;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTFLIP          = 1030;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTJUMP          = 1031;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE        = 1032;
constexpr int32_t ROBOT_SPORT_API_ID_WIGGLEHIPS         = 1033;
constexpr int32_t ROBOT_SPORT_API_ID_GETSTATE           = 1034;
constexpr int32_t ROBOT_SPORT_API_ID_ECONOMICGAIT       = 1035;
constexpr int32_t ROBOT_SPORT_API_ID_HEART              = 1036;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE3             = 1037;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE4             = 1038;
constexpr int32_t ROBOT_SPORT_API_ID_HOPSPINLEFT        = 1039;
constexpr int32_t ROBOT_SPORT_API_ID_HOPSPINRIGHT       = 1040;

constexpr int32_t ROBOT_SPORT_API_ID_LEFTFLIP           = 1042;
constexpr int32_t ROBOT_SPORT_API_ID_BACKFLIP           = 1044;
constexpr int32_t ROBOT_SPORT_API_ID_FREEWALK           = 1045;
constexpr int32_t ROBOT_SPORT_API_ID_FREEBOUND          = 1046;
constexpr int32_t ROBOT_SPORT_API_ID_FREEJUMP           = 1047;
constexpr int32_t ROBOT_SPORT_API_ID_FREEAVOID          = 1048;
constexpr int32_t ROBOT_SPORT_API_ID_WALKSTAIR          = 1049;
constexpr int32_t ROBOT_SPORT_API_ID_WALKUPRIGHT        = 1050;
constexpr int32_t ROBOT_SPORT_API_ID_CROSSSTEP          = 1051;

struct PathPoint
{
    float timeFromStart;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vyaw;
};

class SportClientApi
{
public:
    void DampReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t DampRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void BalanceStandReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t BalanceStandRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void StopMoveReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t StopMoveRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void StandUpReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t StandUpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void StandDownReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t StandDownRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void RecoveryStandReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t RecoveryStandRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void EulerReq(float roll, float pitch, float yaw, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t EulerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void MoveReq(float vx, float vy, float vyaw, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t MoveRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SitReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void RiseSitReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t RiseSitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SwitchGaitReq(int32_t d, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SwitchGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void TriggerReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t TriggerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void BodyHeightReq(float height, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t BodyHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FootRaiseHeightReq(float height, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FootRaiseHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SpeedLevelReq(int32_t level, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SpeedLevelRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void HelloReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t HelloRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void StretchReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t StretchRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void TrajectoryFollowReq(const std::vector<PathPoint>& path, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t TrajectoryFollowRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SwitchJoystickReq(bool on, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SwitchJoystickRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void ContinuousGaitReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t ContinuousGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void WallowReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t WallowRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void ContentReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t ContentRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void HeartReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t HeartRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void PoseReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t PoseRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void ScrapeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t ScrapeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FrontFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FrontFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FrontJumpReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FrontJumpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FrontPounceReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FrontPounceRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void Dance1Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t Dance1Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void Dance2Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t Dance2Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void Dance3Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t Dance3Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void Dance4Req(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t Dance4Res(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void HopSpinLeftReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t HopSpinLeftRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void HopSpinRightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t HopSpinRightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void WiggleHipsReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t WiggleHipsRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void GetStateReq(const std::vector<std::string> &status, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetStateRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res,std::map<std::string, std::string>& statusMap);

    void EconomicGaitReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t EconomicGaitRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void LeftFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t LeftFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void BackFlipReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t BackFlipRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FreeWalkReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FreeWalkRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FreeBoundReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FreeBoundRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FreeJumpReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FreeJumpRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void FreeAvoidReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t FreeAvoidRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void WalkStairReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t WalkStairRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void WalkUprightReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t WalkUprightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void CrossStepReq(bool flag, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t CrossStepRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);
};

}
}
}
