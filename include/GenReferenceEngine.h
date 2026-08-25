#pragma once

#include "gen_obs_builder.h"
#include "gen_params.h"

#include "isaaclab/algorithms/algorithms.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace wbc_deploy {

/// ONNX Gen: proprio hist + velocity waypoints → Arc 39.
class GenReferenceEngine
{
public:
  /// ``infer_log_every``: log ONNX timing every N steps (0 = off).
  explicit GenReferenceEngine(
    const std::filesystem::path& params_dir,
    int infer_log_every = 0);

  void reset();
  void push_proprio(const GenProprioSample& sample);

  /// Run one Gen step; returns Arc length ``output_dim`` (39).
  std::vector<float> step(float vx, float vy, float wz);

  /// Soft torso-height command (cascaded into waypoint features when enabled).
  void set_height_cmd(float height_m);
  void seed_height(float height_m);

  /// Discrete locomotion style for ``wbc_loco_style`` (no-op without ``cmd_style``).
  void set_style(int style_index);
  void set_style(const std::string& style_name);
  void set_style_and_height(int style_index);

  const GenDeployParams& params() const { return params_; }
  bool history_ready() const { return obs_.history_ready(); }
  float height_cmd() const { return obs_.height_cmd(); }
  int style_index() const { return obs_.style_index(); }
  bool has_style() const { return obs_.has_style(); }

private:
  GenDeployParams params_;
  GenObsBuilder obs_;
  std::unique_ptr<isaaclab::OrtRunner> runner_;
};

}  // namespace wbc_deploy
