#include "client/go2/go2_sport_client.hpp"

using namespace unitree::ros2::go2;

#define UT_ROBOT_SPORT_PATH_POINT_SIZE  30
#define SPORT_SERVICE_NAME "sport"
#define SEND_SPORT_REQUEST(REQUEST_FUNC, ...) SEND_REQUEST(mParam, REQUEST_FUNC, __VA_ARGS__)
#define PARSE_SPORT_RESPONSE(RESPONSE_FUNC, ...) PARSE_RESPONSE(mParam, RESPONSE_FUNC, __VA_ARGS__)

SportClient::SportClient(const std::string &nodeName) : BaseClient(nodeName, SPORT_SERVICE_NAME)
{
}

int32_t SportClient::Damp()
{
    SEND_SPORT_REQUEST(DampReq)
    PARSE_SPORT_RESPONSE(DampRes)
}

int32_t SportClient::BalanceStand()
{
    SEND_SPORT_REQUEST(BalanceStandReq)
    PARSE_SPORT_RESPONSE(BalanceStandRes)
}

int32_t SportClient::StopMove()
{
    SEND_SPORT_REQUEST(StopMoveReq)
    PARSE_SPORT_RESPONSE(StopMoveRes)
}

int32_t SportClient::StandUp()
{
    SEND_SPORT_REQUEST(StandUpReq)
    PARSE_SPORT_RESPONSE(StandUpRes)
}

int32_t SportClient::StandDown()
{
    SEND_SPORT_REQUEST(StandDownReq)
    PARSE_SPORT_RESPONSE(StandDownRes)
}

int32_t SportClient::RecoveryStand()
{
    SEND_SPORT_REQUEST(RecoveryStandReq)
    PARSE_SPORT_RESPONSE(RecoveryStandRes)
}

int32_t SportClient::Euler(float roll, float pitch, float yaw)
{
    SEND_SPORT_REQUEST(EulerReq, roll, pitch, yaw)
    PARSE_SPORT_RESPONSE(EulerRes)
}

int32_t SportClient::Move(float vx, float vy, float vyaw)
{
    SEND_SPORT_REQUEST(MoveReq, vx, vy, vyaw)
    PARSE_SPORT_RESPONSE(MoveRes)
}

int32_t SportClient::Sit()
{
    SEND_SPORT_REQUEST(SitReq)
    PARSE_SPORT_RESPONSE(SitRes)
}

int32_t SportClient::RiseSit()
{
    SEND_SPORT_REQUEST(RiseSitReq)
    PARSE_SPORT_RESPONSE(RiseSitRes)
}

int32_t SportClient::SwitchGait(int32_t d)
{
    SEND_SPORT_REQUEST(SwitchGaitReq, d)
    PARSE_SPORT_RESPONSE(SwitchGaitRes)
}

int32_t SportClient::Trigger()
{
    SEND_SPORT_REQUEST(TriggerReq)
    PARSE_SPORT_RESPONSE(TriggerRes)
}

int32_t SportClient::BodyHeight(float height)
{
    SEND_SPORT_REQUEST(BodyHeightReq, height)
    PARSE_SPORT_RESPONSE(BodyHeightRes)
}

int32_t SportClient::FootRaiseHeight(float height)
{
    SEND_SPORT_REQUEST(FootRaiseHeightReq, height)
    PARSE_SPORT_RESPONSE(FootRaiseHeightRes)
}

int32_t SportClient::SpeedLevel(int32_t level)
{
    SEND_SPORT_REQUEST(SpeedLevelReq, level)
    PARSE_SPORT_RESPONSE(SpeedLevelRes)
}

int32_t SportClient::Hello()
{
    SEND_SPORT_REQUEST(HelloReq)
    PARSE_SPORT_RESPONSE(HelloRes)
}

int32_t SportClient::Stretch()
{
    SEND_SPORT_REQUEST(StretchReq)
    PARSE_SPORT_RESPONSE(StretchRes)
}

int32_t SportClient::TrajectoryFollow(const std::vector<PathPoint>& path)
{
    if (path.size() != UT_ROBOT_SPORT_PATH_POINT_SIZE)
    {
        return ERR_ROS_API_INVALID_PARAM;
    }
    SEND_SPORT_REQUEST(TrajectoryFollowReq, path)
    PARSE_SPORT_RESPONSE(TrajectoryFollowRes)
}

int32_t SportClient::SwitchJoystick(bool flag)
{
    SEND_SPORT_REQUEST(SwitchJoystickReq, flag)
    PARSE_SPORT_RESPONSE(SwitchJoystickRes)
}

int32_t SportClient::ContinuousGait(bool flag)
{
    SEND_SPORT_REQUEST(ContinuousGaitReq, flag)
    PARSE_SPORT_RESPONSE(ContinuousGaitRes)
}

int32_t SportClient::Wallow()
{
    SEND_SPORT_REQUEST(WallowReq)
    PARSE_SPORT_RESPONSE(WallowRes)
}

int32_t SportClient::Content()
{
    SEND_SPORT_REQUEST(ContentReq)
    PARSE_SPORT_RESPONSE(ContentRes)
}

int32_t SportClient::Heart()
{
    SEND_SPORT_REQUEST(HeartReq)
    PARSE_SPORT_RESPONSE(HeartRes)
}

int32_t SportClient::Pose(bool flag)
{
    SEND_SPORT_REQUEST(PoseReq, flag)
    PARSE_SPORT_RESPONSE(PoseRes)
}

int32_t SportClient::Scrape()
{
    SEND_SPORT_REQUEST(ScrapeReq)
    PARSE_SPORT_RESPONSE(ScrapeRes)
}

int32_t SportClient::FrontFlip()
{
    SEND_SPORT_REQUEST(FrontFlipReq)
    PARSE_SPORT_RESPONSE(FrontFlipRes)
}

int32_t SportClient::FrontJump()
{
    SEND_SPORT_REQUEST(FrontJumpReq)
    PARSE_SPORT_RESPONSE(FrontJumpRes)
}

int32_t SportClient::FrontPounce()
{
    SEND_SPORT_REQUEST(FrontPounceReq)
    PARSE_SPORT_RESPONSE(FrontPounceRes)
}

int32_t SportClient::Dance1()
{
    SEND_SPORT_REQUEST(Dance1Req)
    PARSE_SPORT_RESPONSE(Dance1Res)
}

int32_t SportClient::Dance2()
{
    SEND_SPORT_REQUEST(Dance2Req)
    PARSE_SPORT_RESPONSE(Dance2Res)
}

int32_t SportClient::Dance3()
{
    SEND_SPORT_REQUEST(Dance3Req)
    PARSE_SPORT_RESPONSE(Dance3Res)
}

int32_t SportClient::Dance4()
{
    SEND_SPORT_REQUEST(Dance4Req)
    PARSE_SPORT_RESPONSE(Dance4Res)
}

int32_t SportClient::HopSpinLeft()
{
    SEND_SPORT_REQUEST(HopSpinLeftReq)
    PARSE_SPORT_RESPONSE(HopSpinLeftRes)
}

int32_t SportClient::HopSpinRight()
{
    SEND_SPORT_REQUEST(HopSpinRightReq)
    PARSE_SPORT_RESPONSE(HopSpinRightRes)
}

int32_t SportClient::WiggleHips()
{
    SEND_SPORT_REQUEST(WiggleHipsReq)
    PARSE_SPORT_RESPONSE(WiggleHipsRes)
}

int32_t SportClient::GetState(const std::vector<std::string>& status, std::map<std::string, std::string>& statusMap)
{
    SEND_SPORT_REQUEST(GetStateReq, status)
    PARSE_SPORT_RESPONSE(GetStateRes, statusMap)
}

int32_t SportClient::EconomicGait(bool flag)
{
    SEND_SPORT_REQUEST(EconomicGaitReq, flag)
    PARSE_SPORT_RESPONSE(EconomicGaitRes)
}

int32_t SportClient::LeftFlip()
{
    SEND_SPORT_REQUEST(LeftFlipReq)
    PARSE_SPORT_RESPONSE(LeftFlipRes)
}

int32_t SportClient::BackFlip()
{
    SEND_SPORT_REQUEST(BackFlipReq)
    PARSE_SPORT_RESPONSE(BackFlipRes)
}

int32_t SportClient::FreeWalk()
{
    SEND_SPORT_REQUEST(BackFlipReq)
    PARSE_SPORT_RESPONSE(BackFlipRes)
}

int32_t SportClient::FreeBound(bool flag)
{
    SEND_SPORT_REQUEST(FreeBoundReq, flag)
    PARSE_SPORT_RESPONSE(FreeBoundRes)
}

int32_t SportClient::FreeJump(bool flag)
{
    SEND_SPORT_REQUEST(FreeJumpReq, flag)
    PARSE_SPORT_RESPONSE(FreeJumpRes)
}

int32_t SportClient::FreeAvoid(bool flag)
{
    SEND_SPORT_REQUEST(FreeAvoidReq, flag)
    PARSE_SPORT_RESPONSE(FreeAvoidRes)
}

int32_t SportClient::WalkStair(bool flag)
{
    SEND_SPORT_REQUEST(WalkStairReq, flag)
    PARSE_SPORT_RESPONSE(WalkStairRes)
}

int32_t SportClient::WalkUpright(bool flag)
{
    SEND_SPORT_REQUEST(WalkUprightReq, flag)
    PARSE_SPORT_RESPONSE(WalkUprightRes)
}

int32_t SportClient::CrossStep(bool flag)
{
    SEND_SPORT_REQUEST(CrossStepReq, flag)
    PARSE_SPORT_RESPONSE(CrossStepRes)
}
