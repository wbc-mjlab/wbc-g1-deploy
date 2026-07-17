// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "FSM/fsm_pose_utils.h"
#include "motion_npz.h"
#include "param.h"
#include "wbc_entry_mode.h"

class State_FloorReady : public FSMState
{
public:
    State_FloorReady(int state, std::string state_string = "FloorReady")
    : FSMState(state, state_string)
    {
        const auto cfg = param::config["FSM"][state_string];
        ts_ = cfg["ts"].as<std::vector<float>>();
        target_q_ = loadGetupFrameZero();
        qs_.resize(2);
        qs_[1] = target_q_;
        assert(ts_.size() == qs_.size());
    }

    void enter()
    {
        static auto kp = param::config["FSM"]["FloorReady"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FloorReady"]["kd"].as<std::vector<float>>();
        fsm_pose::applyMotorGains(kp, kd);
        qs_[0] = fsm_pose::captureMotorPositions(kp.size());
        t0_ = static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
        wbc_deploy::pendingWbcEntryMode() = wbc_deploy::WbcEntryMode::FromFloor;
    }

    void run()
    {
        fsm_pose::runJointInterpolation(t0_, ts_, qs_);
    }

private:
    static std::vector<float> loadGetupFrameZero()
    {
        const auto& root = param::config;
        std::string clip_name = "getup_01";
        if (root["reference_node"] && root["reference_node"]["pose_clips"] &&
            root["reference_node"]["pose_clips"]["getup"]) {
          clip_name =
            root["reference_node"]["pose_clips"]["getup"].as<std::string>();
        } else {
          const auto& wbc_cfg = param::config["FSM"]["Wbc_Tracking"];
          clip_name = wbc_cfg["pose_clips"]["getup"].as<std::string>("getup_01");
        }

        std::filesystem::path clips_dir = param::config_string(
            root, "clips_dir", "config/clips");
        clips_dir = param::resolve_path_under_proj(clips_dir);

        const auto clip_path = resolve_clip_path(clips_dir, clip_name);
        return load_motion_joint_pos_at(clip_path.string(), 0);
    }

    double t0_ = 0.0;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
    std::vector<float> target_q_;
};

REGISTER_FSM(State_FloorReady)
