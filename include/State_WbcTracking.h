#pragma once

#include "FSM/FSMState.h"
#include "MotionClipLibrary.h"
#include "WbcMotionLoader.h"
#include "isaaclab/envs/manager_based_rl_env.h"

#include <functional>
#include <memory>
#include <mutex>


class State_WbcTracking : public FSMState
{
public:
    State_WbcTracking(int state_mode, std::string state_string);
    ~State_WbcTracking();

    void enter();
    void run();
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

    static std::shared_ptr<WbcMotionLoader> motion;

private:
    void handleClipSwitch();
    bool isPlaybackFinished() const;
    void syncPlaybackEndToMotion();
    void applyMotionLoader(bool reset_env);

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::unique_ptr<MotionClipLibrary> clip_library_;
    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;

    bool robot_is_up_ = true;
    bool awaiting_clip_select_ = false;
    bool playback_finished_ = false;
    bool was_playback_finished_ = false;

    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_next_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_prev_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_getup_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_liedown_fn_;
    std::mutex tracking_mtx_;
};


REGISTER_FSM(State_WbcTracking)
