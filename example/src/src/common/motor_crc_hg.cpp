#include "motor_crc_hg.h"
#include "utils/crc.hpp"

void get_crc(unitree_hg::msg::LowCmd &msg)
{
    LowCmd raw{};

    raw.modePr = msg.mode_pr;
    raw.modeMachine = msg.mode_machine;

    for (int i = 0; i < 35; i++)
    {
        raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
        raw.motorCmd[i].q = msg.motor_cmd[i].q;
        raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
        raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
        raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
        raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

        raw.motorCmd[i].reserve = msg.motor_cmd[i].reserve;
    }

    memcpy(&raw.reserve[0], &msg.reserve[0], 4);

    raw.crc = unitree::common::crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
    msg.crc = raw.crc;
}
