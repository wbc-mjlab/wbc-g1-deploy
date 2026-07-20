#pragma once

#include "FSM/FSMState.h"
#include "IMotionReference.h"
#include "MotionClipLibrary.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "pd_torque_clip.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>


class State_WbcTracking : public FSMState
{
public:
    State_WbcTracking(int state_mode, std::string state_string);
    ~State_WbcTracking();

    void enter();
    void run();
    void exit();

    /// Active reference source for MDP ``ref_*`` + residual ``q_ref``.
    static std::shared_ptr<IMotionReference> motion;

private:
    void handleClipSwitch();
    bool isPlaybackFinished() const;
    float currentPlaybackTime() const;
    void syncPlaybackEndToMotion();
    void applyMotionLoader(bool reset_env);
    void beginClipPlayback();
    void pauseClipPlayback(float frozen_time);
    void enterClipSelectMode(float frozen_time);
    /// After FloorReady → WBC: seed / hold getup frame 0 on the DDS reference
    /// until the ref node starts a getup episode.
    bool seedDdsGetupFrameZeroHold();
    void reapplyDdsFloorHoldIfNeeded();

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::unique_ptr<MotionClipLibrary> clip_library_;
    std::string reference_source_ = "clips";  // clips | dds
    std::string anchor_body_ = "torso_link";
    float step_dt_ = 0.02f;
    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;

    bool robot_is_up_ = true;
    bool awaiting_clip_select_ = false;
    bool clip_playback_active_ = false;
    float frozen_playback_time_ = 0.0f;
    bool playback_finished_ = false;
    bool was_playback_finished_ = false;

    /// DDS FromFloor: keep publishing getup frame 0 locally until getup plays.
    bool dds_floor_hold_active_ = false;
    std::vector<float> dds_floor_hold_arc_;

    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_next_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_prev_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_play_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_getup_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_liedown_fn_;
    wbc_deploy::PdTorqueClipConfig pd_torque_clip_;
    std::mutex tracking_mtx_;
};


REGISTER_FSM(State_WbcTracking)
