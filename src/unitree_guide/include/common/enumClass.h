/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform
{
    GAZEBO,
    REALROBOT,
};

enum class RobotType
{
    A1,
    Go1,
    B1
};

enum class UserCommand
{
    // EXIT,
    NONE,
    START,     // trotting
    START_MPC, // mpc
    L2_A,      // fixedStand
    L2_B,      // passive
    L2_X,      // freeStand
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y, // move_base
#endif    // COMPILE_WITH_MOVE_BASE
    L1_X, // balanceTest
    L1_A, // swingTest
    L1_Y  // stepTest
};

enum class FrameType
{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus
{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

enum class FSMStateName
{
    // EXIT,
    INVALID, // 0
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
    MPC,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE, // move_base
#endif         // COMPILE_WITH_MOVE_BASE
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

#endif // ENUMCLASS_H