// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "FSM/fsm_pose_utils.h"
#include "wbc_entry_mode.h"

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
    }

    void enter()
    {
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        fsm_pose::applyMotorGains(kp, kd);
        qs_[0] = fsm_pose::captureMotorPositions(kp.size());
        t0_ = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        wbc_deploy::pendingWbcEntryMode() = wbc_deploy::WbcEntryMode::Standing;
    }

    void run()
    {
        fsm_pose::runJointInterpolation(t0_, ts_, qs_);
    }

private:
    double t0_;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_FixStand)
