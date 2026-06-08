#pragma once

#include "FSM/FSMState.h"
#include "MotionClipLibrary.h"
#include "WbcMotionLoader.h"
#include "isaaclab/envs/manager_based_rl_env.h"

#include <functional>
#include <memory>


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

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::unique_ptr<MotionClipLibrary> clip_library_;
    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;
    float env_origin_z_ = 0.0f;
    bool has_state_estimation_ = false;

    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_next_fn_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> clip_prev_fn_;
};


REGISTER_FSM(State_WbcTracking)
