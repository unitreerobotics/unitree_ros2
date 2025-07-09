#include "client/g1/g1_loco_client.hpp"

using namespace unitree::ros2::g1;

#define LOCO_SERVICE_NAME "sport"
#define SEND_LOCO_REQUEST(REQUEST_FUNC, ...) SEND_REQUEST(mParam, REQUEST_FUNC, __VA_ARGS__)
#define PARSE_LOCO_RESPONSE(RESPONSE_FUNC, ...) PARSE_RESPONSE(mParam, RESPONSE_FUNC, __VA_ARGS__)

LocoClient::LocoClient(const std::string &nodeName) : BaseClient(nodeName, LOCO_SERVICE_NAME)
{
}

int32_t LocoClient::GetFsmId(int32_t &fsm_id)
{
    SEND_LOCO_REQUEST(GetFsmIdReq)
    PARSE_LOCO_RESPONSE(GetFsmIdRes, fsm_id)
}

int32_t LocoClient::GetFsmMode(int32_t &fsm_mode)
{
    SEND_LOCO_REQUEST(GetFsmModeReq)
    PARSE_LOCO_RESPONSE(GetFsmModeRes, fsm_mode)
}

int32_t LocoClient::GetBalanceMode(int32_t& balance_mode)
{
    SEND_LOCO_REQUEST(GetBalanceModeReq)
    PARSE_LOCO_RESPONSE(GetBalanceModeRes, balance_mode)
}

int32_t LocoClient::GetSwingHeight(float& swing_height)
{
    SEND_LOCO_REQUEST(GetSwingHeightReq)
    PARSE_LOCO_RESPONSE(GetSwingHeightRes, swing_height)
}

int32_t LocoClient::GetStandHeight(float& stand_height)
{
    SEND_LOCO_REQUEST(GetStandHeightReq)
    PARSE_LOCO_RESPONSE(GetStandHeightRes, stand_height)
}

int32_t LocoClient::GetPhase(std::vector<float>& phase)
{
    SEND_LOCO_REQUEST(GetPhaseReq)
    PARSE_LOCO_RESPONSE(GetPhaseRes, phase)
}

int32_t LocoClient::SetFsmId(int32_t fsmId)
{
    SEND_LOCO_REQUEST(SetFsmIdReq,fsmId)
    PARSE_LOCO_RESPONSE(SetFsmIdRes)
}

int32_t LocoClient::SetBalanceMode(int32_t balanceMode)
{
    SEND_LOCO_REQUEST(SetBalanceModeReq,balanceMode)
    PARSE_LOCO_RESPONSE(SetBalanceModeRes)
}

int32_t LocoClient::SetSwingHeight(float swingHeight)
{
    SEND_LOCO_REQUEST(SetSwingHeightReq,swingHeight)
    PARSE_LOCO_RESPONSE(SetSwingHeightRes)
}

int32_t LocoClient::SetStandHeight(float standHeight)
{
    SEND_LOCO_REQUEST(SetStandHeightReq,standHeight)
    PARSE_LOCO_RESPONSE(SetStandHeightRes)
}

int32_t LocoClient::SetVelocity(float vx, float vy, float omega, float duration)
{
    SEND_LOCO_REQUEST(SetVelocityReq, vx, vy, omega, duration)
    PARSE_LOCO_RESPONSE(SetVelocityRes)
}

int32_t LocoClient::SetTaskId(int32_t taskId)
{
    SEND_LOCO_REQUEST(SetTaskIdReq, taskId)
    PARSE_LOCO_RESPONSE(SetTaskIdRes)
}

int32_t LocoClient::Damp()
{
    return SetFsmId(1);
}

int32_t LocoClient::Start()
{
    return SetFsmId(200);
}

int32_t LocoClient::Squat()
{
    return SetFsmId(2);
}

int32_t LocoClient::Sit()
{
    return SetFsmId(3);
}

int32_t LocoClient::StandUp()
{
    return SetFsmId(4);
}

int32_t LocoClient::ZeroTorque()
{
    return SetFsmId(0);
}

int32_t LocoClient::StopMove()
{
    return SetVelocity(0.f, 0.f, 0.f);
}

int32_t LocoClient::HighStand()
{
    return SetStandHeight(std::numeric_limits<uint32_t>::max());
}

int32_t LocoClient::LowStand()
{
    return SetStandHeight(std::numeric_limits<uint32_t>::min());
}

int32_t LocoClient::Move(float vx, float vy, float vyaw, bool continous_move)
{
    return SetVelocity(vx, vy, vyaw, continous_move ? 864000.f : 1.f);
}

int32_t LocoClient::Move(float vx, float vy, float vyaw)
{
    return Move(vx, vy, vyaw, mContinousMove);
}

int32_t LocoClient::SwitchMoveMode(bool flag)
{
    mContinousMove = flag;
    return 0;
}

int32_t LocoClient::BalanceStand()
{
    return SetBalanceMode(0);
}

int32_t LocoClient::ContinuousGait(bool flag)
{
    return SetBalanceMode(flag ? 1 : 0);
}

int32_t LocoClient::WaveHand(bool turn_flag)
{
    return SetTaskId(turn_flag ? 1 : 0);
}

int32_t LocoClient::ShakeHand(int stage)
{
    switch (stage)
    {
    case 0:
        mFirstShakeHandStage = false;
        return SetTaskId(2);
    case 1:
        mFirstShakeHandStage = true;
        return SetTaskId(3);
    default:
        mFirstShakeHandStage = !mFirstShakeHandStage;
        return SetTaskId(mFirstShakeHandStage ? 3 : 2);
    }
}
