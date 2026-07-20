#pragma once

#include "gen_params.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace wbc_deploy {

/// Single-frame proprio sample. Keys are **deploy** state names from
/// ``params/config.yaml`` (e.g. ``joint_pos_rel``, ``joint_torque``).
///
/// Fill all available sensors once; ``GenObsBuilder`` only packs terms listed
/// in the YAML so train/export can add or remove terms without C++ changes
/// beyond providing the sensor values here.
struct GenProprioSample {
  std::unordered_map<std::string, std::vector<float>> terms;

  void set(std::string name, std::vector<float> values)
  {
    terms[std::move(name)] = std::move(values);
  }

  const std::vector<float>* get(const std::string& name) const
  {
    const auto it = terms.find(name);
    return it == terms.end() ? nullptr : &it->second;
  }
};

/// Standing / zero proprio for every term present in ``params``.
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
  GenDeployParams params_;
  std::unordered_map<std::string, std::deque<std::vector<float>>> rings_;
  bool history_ready_ = false;
  float height_cmd_ = 0.80f;
  std::vector<float> height_wp_;
};

}  // namespace wbc_deploy
