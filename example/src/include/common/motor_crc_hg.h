/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_H_
#define _MOTOR_CRC_H_

#include <stdint.h>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

constexpr int LeftHipPitch = 0;
constexpr int LeftHipRoll = 1;
constexpr int LeftHipYaw = 2;
constexpr int LeftKnee = 3;
constexpr int LeftAnklePitch = 4;
constexpr int LeftAnkleB = 4;
constexpr int LeftAnkleRoll = 5;
constexpr int LeftAnkleA = 5;
constexpr int RightHipPitch = 6;
constexpr int RightHipRoll = 7;
constexpr int RightHipYaw = 8;
constexpr int RightKnee = 9;
constexpr int RightAnklePitch = 10;
constexpr int RightAnkleB = 10;
constexpr int RightAnkleRoll = 11;
constexpr int RightAnkleA = 11;
constexpr int WaistYaw = 12;
constexpr int WaistRoll = 13;	 // NOTE INVALID for g1 23dof/29dof with waist locked
constexpr int WaistA = 13;	 // NOTE INVALID for g1 23dof/29dof with waist locked
constexpr int WaistPitch = 14; // NOTE INVALID for g1 23dof/29dof with waist locked
constexpr int WaistB = 14;	 // NOTE INVALID for g1 23dof/29dof with waist locked
constexpr int LeftShoulderPitch = 15;
constexpr int LeftShoulderRoll = 16;
constexpr int LeftShoulderYaw = 17;
constexpr int LeftElbow = 18;
constexpr int LeftWristRoll = 19;
constexpr int LeftWristPitch = 20; // NOTE INVALID for g1 23dof
constexpr int LeftWristYaw = 21;	 // NOTE INVALID for g1 23dof
constexpr int RightShoulderPitch = 22;
constexpr int RightShoulderRoll = 23;
constexpr int RightShoulderYaw = 24;
constexpr int RightElbow = 25;
constexpr int RightWristRoll = 26;
constexpr int RightWristPitch = 27; // NOTE INVALID for g1 23dof
constexpr int RightWristYaw = 28;	  // NOTE INVALID for g1 23dof

typedef struct
{
	uint8_t mode; // desired working mode
	float q;	  // desired angle (unit: radian)
	float dq;	  // desired velocity (unit: radian/second)
	float tau;	  // desired output torque (unit: N.m)
	float Kp;	  // desired position stiffness (unit: N.m/rad )
	float Kd;	  // desired velocity stiffness (unit: N.m/(rad/s) )
	uint32_t reserve = 0;
} MotorCmd; // motor control

typedef struct
{
	uint8_t modePr;
	uint8_t modeMachine;
	std::array<MotorCmd, 35> motorCmd;
	std::array<uint32_t, 4> reserve;
	uint32_t crc;
} LowCmd;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void get_crc(unitree_hg::msg::LowCmd &msg);

#endif