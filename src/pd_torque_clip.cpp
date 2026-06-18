#include "pd_torque_clip.h"

#include "param.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace wbc_deploy {
namespace {

YAML::Node load_yaml_file(const std::filesystem::path& path)
{
  return YAML::LoadFile(path.string());
}

YAML::Node load_robot_defaults()
{
  for (const char* name : {"robot_defaults.yaml", "policy_defaults.yaml"}) {
    const auto path = param::config_dir / name;
    if (std::filesystem::exists(path)) {
      return load_yaml_file(path);
    }
  }
  return YAML::Node(YAML::NodeType::Map);
}

bool try_pick_node(
  const YAML::Node& state_cfg,
  const YAML::Node& defaults,
  const char* key,
  YAML::Node& out)
{
  if (state_cfg["pd_torque_clipping"] && state_cfg["pd_torque_clipping"][key]) {
    out = state_cfg["pd_torque_clipping"][key];
    return true;
  }
  if (defaults["pd_torque_clipping"] && defaults["pd_torque_clipping"][key]) {
    out = defaults["pd_torque_clipping"][key];
    return true;
  }
  return false;
}

}  // namespace

PdTorqueClipConfig load_pd_torque_clip_config(
  const YAML::Node& state_cfg,
  size_t num_joints)
{
  const YAML::Node defaults = load_robot_defaults();
  PdTorqueClipConfig cfg;
  YAML::Node value;
  if (try_pick_node(state_cfg, defaults, "enabled", value)) {
    cfg.enabled = value.as<bool>();
  }
  if (try_pick_node(state_cfg, defaults, "fraction", value)) {
    cfg.fraction = value.as<float>();
  }
  if (try_pick_node(state_cfg, defaults, "joint_max_torque", value)) {
    cfg.joint_max_torque = value.as<std::vector<float>>();
  }
  if (cfg.fraction <= 0.0f || cfg.fraction > 1.0f) {
    throw std::runtime_error("pd_torque_clipping.fraction must be in (0, 1]");
  }
  if (cfg.enabled && cfg.joint_max_torque.size() != num_joints) {
    throw std::runtime_error(
      "pd_torque_clipping.joint_max_torque size "
      + std::to_string(cfg.joint_max_torque.size())
      + " does not match joint count "
      + std::to_string(num_joints));
  }
  log_pd_torque_clip_status(cfg);
  return cfg;
}

void log_pd_torque_clip_status(const PdTorqueClipConfig& cfg)
{
  if (!cfg.enabled) {
    const std::string msg =
      "PD torque clipping: OFF (WBC position targets sent without torque back-solve)";
    spdlog::info(msg);
    std::cout << msg << '\n';
    return;
  }

  const auto [min_it, max_it] = std::minmax_element(
    cfg.joint_max_torque.begin(), cfg.joint_max_torque.end());
  const std::string msg = fmt::format(
    "PD torque clipping: ON — |tau| <= {:.0f}% of catalog peak "
    "({} joints, {:.1f}–{:.1f} N·m)",
    cfg.fraction * 100.0f,
    cfg.joint_max_torque.size(),
    *min_it,
    *max_it);
  spdlog::info(msg);
  std::cout << msg << '\n';
}

float clip_pd_torque_position(
  float q_des,
  float q_cur,
  float dq_cur,
  float kp,
  float kd,
  float tau_limit,
  float fraction,
  bool enabled)
{
  if (!enabled || tau_limit <= 0.0f) {
    return q_des;
  }
  if (std::abs(kp) < 1e-9f) {
    return q_cur;
  }

  auto implied_torque = [&](float q) {
    return kp * (q - q_cur) - kd * dq_cur;
  };

  float torque = implied_torque(q_des);
  const float tau_cap = fraction * tau_limit;

  if (std::abs(torque) > tau_cap) {
    const float tau_clipped = std::clamp(torque, -tau_cap, tau_cap);
    q_des = q_cur + (tau_clipped + kd * dq_cur) / kp;
    torque = implied_torque(q_des);
  }

  if (std::abs(torque) > tau_limit) {
    const float tau_clipped = std::clamp(torque, -tau_limit, tau_limit);
    q_des = q_cur + (tau_clipped + kd * dq_cur) / kp;
  }

  return q_des;
}

void clip_pd_torque_positions(
  std::vector<float>& q_des,
  const std::vector<float>& q_cur,
  const std::vector<float>& dq_cur,
  const std::vector<float>& kp,
  const std::vector<float>& kd,
  const PdTorqueClipConfig& cfg)
{
  if (!cfg.enabled) {
    return;
  }
  const size_t n = q_des.size();
  if (q_cur.size() != n || dq_cur.size() != n || kp.size() != n || kd.size() != n
      || cfg.joint_max_torque.size() != n) {
    throw std::runtime_error("clip_pd_torque_positions: size mismatch");
  }

  for (size_t i = 0; i < n; ++i) {
    const float before = q_des[i];
    q_des[i] = clip_pd_torque_position(
      q_des[i],
      q_cur[i],
      dq_cur[i],
      kp[i],
      kd[i],
      cfg.joint_max_torque[i],
      cfg.fraction,
      true);
    if (std::abs(q_des[i] - before) > 1e-5f) {
      spdlog::debug(
        "PD torque clip joint {}: q {:.4f} -> {:.4f}",
        i,
        before,
        q_des[i]);
    }
  }
}

}  // namespace wbc_deploy
