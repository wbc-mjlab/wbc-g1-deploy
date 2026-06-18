#pragma once

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <vector>

namespace wbc_deploy {

/// PD-implied torque clipping (back-solve position), matching robot_core motor_control_cpp.
struct PdTorqueClipConfig
{
  bool enabled = false;
  float fraction = 0.98f;
  std::vector<float> joint_max_torque;
};

PdTorqueClipConfig load_pd_torque_clip_config(
  const YAML::Node& state_cfg,
  size_t num_joints);

float clip_pd_torque_position(
  float q_des,
  float q_cur,
  float dq_cur,
  float kp,
  float kd,
  float tau_limit,
  float fraction,
  bool enabled);

void clip_pd_torque_positions(
  std::vector<float>& q_des,
  const std::vector<float>& q_cur,
  const std::vector<float>& dq_cur,
  const std::vector<float>& kp,
  const std::vector<float>& kd,
  const PdTorqueClipConfig& cfg);

void log_pd_torque_clip_status(const PdTorqueClipConfig& cfg);

}  // namespace wbc_deploy
