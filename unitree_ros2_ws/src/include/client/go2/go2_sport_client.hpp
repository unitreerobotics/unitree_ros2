#pragma once

#include "client/base_client.hpp"
#include "client/go2/go2_sport_api.hpp"

namespace unitree
{
namespace ros2
{
namespace go2
{

class SportClient: public BaseClient
{
public:
    SportClient(const std::string &nodeName = "go2_sport_lient");

    int32_t Damp();

    int32_t BalanceStand();

    int32_t StopMove();

    int32_t StandUp();

    int32_t StandDown();

    int32_t RecoveryStand();

    int32_t Euler(float roll, float pitch, float yaw);

    int32_t Move(float vx, float vy, float vyaw);

    int32_t Sit();

    int32_t RiseSit();

    int32_t SwitchGait(int32_t d);

    int32_t Trigger();

    int32_t BodyHeight(float height);

    int32_t FootRaiseHeight(float height);

    int32_t SpeedLevel(int32_t level);

    int32_t Hello();

    int32_t Stretch();

    int32_t TrajectoryFollow(const std::vector<PathPoint>& path);

    int32_t SwitchJoystick(bool flag);

    int32_t ContinuousGait(bool flag);

    int32_t Wallow();

    int32_t Content();

    int32_t Heart();

    int32_t Pose(bool flag);

    int32_t Scrape();

    int32_t FrontFlip();

    int32_t FrontJump();

    int32_t FrontPounce();

    int32_t Dance1();

    int32_t Dance2();

    int32_t Dance3();

    int32_t Dance4();

    int32_t HopSpinLeft();

    int32_t HopSpinRight();

    int32_t WiggleHips();

    int32_t GetState(const std::vector<std::string>& status, std::map<std::string, std::string>& statusMap);

    int32_t EconomicGait(bool flag);

    int32_t LeftFlip();

    int32_t BackFlip();

    int32_t FreeWalk();

    int32_t FreeBound(bool flag);

    int32_t FreeJump(bool flag);

    int32_t FreeAvoid(bool flag);

    int32_t WalkStair(bool flag);

    int32_t WalkUpright(bool flag);

    int32_t CrossStep(bool flag);
private:
    SportClientApi mParam;
};

}
}
}
