#pragma once

#include "gen_params.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace wbc_deploy {

/// Single-frame proprio sample matching Gen state terms (no history).
struct GenProprioSample {
  std::vector<float> base_ang_vel;       // 3
  std::vector<float> projected_gravity;  // 3
  std::vector<float> joint_pos_rel;      // 29
  std::vector<float> joint_vel_rel;      // 29
};

/// Standing / zero proprio (at default pose).
GenProprioSample standing_proprio_sample(const GenDeployParams& params);

/// History rings + flat state ‖ command packing for ``generator.onnx``.
class GenObsBuilder
{
public:
  explicit GenObsBuilder(GenDeployParams params);

  void reset(const GenProprioSample& fill);
  void push(const GenProprioSample& sample);

  /// Desired torso / link height (soft-cascaded into waypoint features).
  void set_height_cmd(float height_cmd);
  void seed_height(float height);

  /// Flat ``state ‖ command`` vector of size ``input_dim``.
  /// Advances the height cascade one control step when height features are enabled.
  std::vector<float> build_obs(float vx, float vy, float wz);

  const GenDeployParams& params() const { return params_; }
  bool history_ready() const { return history_ready_; }
  float height_cmd() const { return height_cmd_; }

private:
  const std::vector<float>& term_values(
    const GenProprioSample& sample,
    const std::string& deploy_name) const;

  GenDeployParams params_;
  std::unordered_map<std::string, std::deque<std::vector<float>>> rings_;
  bool history_ready_ = false;
  float height_cmd_ = 0.80f;
  std::vector<float> height_wp_;
};

}  // namespace wbc_deploy
