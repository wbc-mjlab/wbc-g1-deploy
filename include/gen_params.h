#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace wbc_deploy {

struct GenStateTermCfg {
  std::string deploy_name;
  std::string training_name;
  int dim = 0;
  int history_length = 1;
  int flat_width = 0;
  std::vector<float> scale;
};

struct GenCommandTermCfg {
  std::string deploy_name;
  std::string training_name;
  int dim = 0;
  int history_length = 0;
  int flat_width = 0;
  std::vector<float> scale;
  std::vector<int> horizons;
  float position_scale = 1.0f;
  float height_scale = 1.0f;
  float height_lowpass_tau = 0.0f;
  int height_setpoint_dim = 0;
  int features_per_horizon = 0;
};

struct GenCommandCfg {
  std::vector<std::string> term_names;
  std::unordered_map<std::string, GenCommandTermCfg> observations;
  std::vector<int> horizons;
  float position_scale = 1.0f;
  float height_scale = 1.0f;
  float height_lowpass_tau = 0.10f;
  /// Derived convenience for teleop (0 = no setpoint term).
  int height_setpoint_dim = 0;
  /// Derived convenience for teleop (0 = no per-horizon height).
  int height_features_per_horizon = 0;
};

struct GenModelCfg {
  std::string onnx_file = "generator.onnx";
  std::string onnx_input_name = "obs";
  std::string onnx_output_name = "reference";
  /// Backend used at training / ONNX export (``flow_ae``, ``cascade_ae``, …).
  std::string type;
  /// Optional metadata (present depending on ``type``).
  int n_euler_steps = 0;
  int latent_dim = 0;
  int intent_dim = 0;
  bool refine = false;
  bool use_ae = false;
  int history_length = 0;
};

struct GenDeployParams {
  std::string schema_version;
  std::string robot_id = "g1";
  std::string overlay;
  float step_dt = 0.02f;
  std::vector<std::string> joint_names;
  std::vector<int> joint_ids_map;
  std::vector<float> default_joint_pos;
  std::vector<std::string> state_observation_names;
  std::unordered_map<std::string, GenStateTermCfg> state_observations;
  int history_length = 1;
  GenCommandCfg command;
  int input_dim = 0;
  int state_dim = 0;
  int command_dim = 0;
  int output_dim = 39;
  GenModelCfg model;
  std::unordered_map<std::string, std::pair<float, float>> play_vel_ranges;
  std::filesystem::path params_dir;
};

/// Load ``wbc_gen_deploy_params_v2`` from ``params/config.yaml``.
GenDeployParams load_gen_deploy_params(const std::filesystem::path& params_dir);

std::filesystem::path resolve_generator_onnx(const GenDeployParams& params);

}  // namespace wbc_deploy
