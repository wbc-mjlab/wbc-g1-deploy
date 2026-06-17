#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"

#include <vector>

namespace fsm_pose {

inline void applyMotorGains(
  const std::vector<float>& kp,
  const std::vector<float>& kd)
{
  for (size_t i = 0; i < kp.size(); ++i) {
    auto& motor = FSMState::lowcmd->msg_.motor_cmd()[i];
    motor.kp() = kp[i];
    motor.kd() = kd[i];
    motor.dq() = motor.tau() = 0;
  }
}

inline std::vector<float> captureMotorPositions(size_t count)
{
  std::vector<float> q0;
  q0.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    q0.push_back(FSMState::lowcmd->msg_.motor_cmd()[i].q());
  }
  return q0;
}

inline void runJointInterpolation(
  double t0,
  const std::vector<float>& ts,
  const std::vector<std::vector<float>>& qs)
{
  const float t = static_cast<float>(
    static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3 - t0);
  const auto q = linear_interpolate(t, ts, qs);
  for (size_t i = 0; i < q.size(); ++i) {
    FSMState::lowcmd->msg_.motor_cmd()[i].q() = q[i];
  }
}

}  // namespace fsm_pose
