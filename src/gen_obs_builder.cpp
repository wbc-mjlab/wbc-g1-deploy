#include "gen_obs_builder.h"

#include "gen_waypoints.h"

#include <algorithm>
#include <stdexcept>

namespace wbc_deploy {

namespace {

/// Match play ``DEFAULT_STAND_HEIGHT`` (0.80 m), clamped to configured range.
float default_stand_height(const GenDeployParams& params)
{
  constexpr float kIdle = 0.80f;
  const auto it = params.play_vel_ranges.find("height");
  if (it != params.play_vel_ranges.end()) {
    return std::clamp(kIdle, it->second.first, it->second.second);
  }
  return kIdle;
}

std::vector<float> zero_term(int dim, const std::string& name)
{
  if (name == "projected_gravity" && dim >= 3) {
    return {0.0f, 0.0f, -1.0f};
  }
  return std::vector<float>(static_cast<size_t>(std::max(dim, 0)), 0.0f);
}

}  // namespace

GenProprioSample standing_proprio_sample(const GenDeployParams& params)
{
  GenProprioSample s;
  for (const auto& name : params.state_observation_names) {
    const auto& cfg = params.state_observations.at(name);
    s.set(name, zero_term(cfg.dim, name));
  }
  return s;
}

GenObsBuilder::GenObsBuilder(GenDeployParams params)
: params_(std::move(params))
{
  for (const auto& name : params_.state_observation_names) {
    rings_[name] = {};
  }
  const float h0 = default_stand_height(params_);
  seed_height(h0);
  reset(standing_proprio_sample(params_));
}

void GenObsBuilder::seed_height(float height)
{
  height_cmd_ = height;
  const int k = static_cast<int>(params_.command.horizons.size());
  height_wp_.assign(static_cast<size_t>(std::max(k, 0)), height);
}

void GenObsBuilder::set_height_cmd(float height_cmd)
{
  height_cmd_ = height_cmd;
}

void GenObsBuilder::reset(const GenProprioSample& fill)
{
  for (const auto& name : params_.state_observation_names) {
    const auto& cfg = params_.state_observations.at(name);
    const auto* vals_ptr = fill.get(name);
    if (vals_ptr == nullptr) {
      throw std::runtime_error(
        "Proprio sample missing term required by config: " + name);
    }
    const auto& vals = *vals_ptr;
    if (static_cast<int>(vals.size()) != cfg.dim) {
      throw std::runtime_error(
        "Proprio term " + name + " dim mismatch: got " +
        std::to_string(vals.size()) + " expected " + std::to_string(cfg.dim));
    }
    auto& ring = rings_[name];
    ring.clear();
    for (int i = 0; i < cfg.history_length; ++i) {
      ring.push_back(vals);
    }
  }
  history_ready_ = true;
  seed_height(default_stand_height(params_));
}

void GenObsBuilder::push(const GenProprioSample& sample)
{
  for (const auto& name : params_.state_observation_names) {
    const auto& cfg = params_.state_observations.at(name);
    const auto* vals_ptr = sample.get(name);
    if (vals_ptr == nullptr) {
      throw std::runtime_error(
        "Proprio sample missing term required by config: " + name);
    }
    auto vals = *vals_ptr;
    if (static_cast<int>(vals.size()) != cfg.dim) {
      throw std::runtime_error("Proprio term " + name + " dim mismatch on push");
    }
    for (size_t i = 0; i < vals.size() && i < cfg.scale.size(); ++i) {
      vals[i] *= cfg.scale[i];
    }
    auto& ring = rings_[name];
    ring.push_back(std::move(vals));
    while (static_cast<int>(ring.size()) > cfg.history_length) {
      ring.pop_front();
    }
  }
  history_ready_ = true;
  for (const auto& name : params_.state_observation_names) {
    if (static_cast<int>(rings_.at(name).size()) <
        params_.state_observations.at(name).history_length) {
      history_ready_ = false;
      break;
    }
  }
}

std::vector<float> GenObsBuilder::build_obs(float vx, float vy, float wz)
{
  std::vector<float> obs;
  obs.reserve(static_cast<size_t>(params_.input_dim));

  for (const auto& name : params_.state_observation_names) {
    const auto& cfg = params_.state_observations.at(name);
    const auto& ring = rings_.at(name);
    if (static_cast<int>(ring.size()) != cfg.history_length) {
      throw std::runtime_error("History not ready for term " + name);
    }
    // Oldest → newest (mjlab flatten_history_dim).
    for (const auto& frame : ring) {
      obs.insert(obs.end(), frame.begin(), frame.end());
    }
  }

  std::vector<float> xy;
  std::vector<float> ang;
  integrate_vel_to_sparse_waypoints(
    vx,
    vy,
    wz,
    params_.command.horizons,
    params_.step_dt,
    params_.command.position_scale,
    xy,
    ang);

  std::vector<float> height;
  if (params_.command.height_features_per_horizon > 0) {
    if (height_wp_.size() != params_.command.horizons.size()) {
      seed_height(height_cmd_);
    }
    lowpass_height_waypoints(
      height_wp_,
      height_cmd_,
      params_.step_dt,
      params_.command.horizons,
      params_.command.height_lowpass_tau);
    height = height_wp_;
  }

  const auto cmd = pack_command_xy_height_angle(xy, height, ang);
  obs.insert(obs.end(), cmd.begin(), cmd.end());

  if (static_cast<int>(obs.size()) != params_.input_dim) {
    throw std::runtime_error(
      "Built obs dim " + std::to_string(obs.size()) +
      " != input_dim " + std::to_string(params_.input_dim));
  }
  return obs;
}

}  // namespace wbc_deploy
