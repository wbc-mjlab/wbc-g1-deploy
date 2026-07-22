#include "gen_params.h"

#include <spdlog/spdlog.h>

#include <stdexcept>

namespace wbc_deploy {

namespace {

std::vector<float> as_float_list(const YAML::Node& node, int fallback_dim)
{
  std::vector<float> out;
  if (!node) {
    out.assign(static_cast<size_t>(std::max(fallback_dim, 0)), 1.0f);
    return out;
  }
  if (node.IsSequence()) {
    for (const auto& v : node) {
      out.push_back(v.as<float>());
    }
    return out;
  }
  out.assign(static_cast<size_t>(std::max(fallback_dim, 1)), node.as<float>());
  return out;
}

}  // namespace

GenDeployParams load_gen_deploy_params(const std::filesystem::path& params_dir)
{
  const auto cfg_path = params_dir / "config.yaml";
  if (!std::filesystem::exists(cfg_path)) {
    throw std::runtime_error("Gen params config missing: " + cfg_path.string());
  }
  const YAML::Node doc = YAML::LoadFile(cfg_path.string());
  GenDeployParams p;
  p.params_dir = params_dir;
  p.schema_version = doc["schema_version"].as<std::string>("");
  if (p.schema_version != "wbc_gen_deploy_params_v1") {
    throw std::runtime_error(
      "Expected schema_version wbc_gen_deploy_params_v1, got '" + p.schema_version + "'");
  }
  p.robot_id = doc["robot_id"].as<std::string>("g1");
  p.overlay = doc["overlay"].as<std::string>("");
  p.step_dt = doc["step_dt"].as<float>(0.02f);

  if (doc["joint_names"]) {
    for (const auto& n : doc["joint_names"]) {
      p.joint_names.push_back(n.as<std::string>());
    }
  }
  if (doc["joint_ids_map"]) {
    for (const auto& n : doc["joint_ids_map"]) {
      p.joint_ids_map.push_back(n.as<int>());
    }
  }
  if (doc["default_joint_pos"]) {
    for (const auto& n : doc["default_joint_pos"]) {
      p.default_joint_pos.push_back(n.as<float>());
    }
  }

  const auto state = doc["state"];
  if (!state) {
    throw std::runtime_error("Gen params missing 'state' block");
  }
  p.history_length = state["history_length"].as<int>(1);
  for (const auto& n : state["observation_names"]) {
    p.state_observation_names.push_back(n.as<std::string>());
  }
  const auto obs = state["observations"];
  for (const auto& name : p.state_observation_names) {
    if (!obs || !obs[name]) {
      throw std::runtime_error("Gen params missing state.observations." + name);
    }
    const auto term = obs[name];
    GenStateTermCfg cfg;
    cfg.deploy_name = name;
    cfg.training_name = term["training_name"].as<std::string>(name);
    cfg.dim = term["dim"].as<int>();
    cfg.history_length = term["history_length"].as<int>(p.history_length);
    cfg.flat_width = term["flat_width"].as<int>(cfg.dim * cfg.history_length);
    cfg.scale = as_float_list(term["scale"], cfg.dim);
    p.state_observations[name] = std::move(cfg);
  }

  const auto cmd = doc["command"];
  if (!cmd) {
    throw std::runtime_error("Gen params missing 'command' block");
  }
  p.command.packing = cmd["packing"].as<std::string>("xy_then_height_then_angle");
  for (const auto& h : cmd["horizons"]) {
    p.command.horizons.push_back(h.as<int>());
  }
  p.command.position_scale = cmd["position_scale"].as<float>(1.0f);
  p.command.xy_features_per_horizon = cmd["xy_features_per_horizon"].as<int>(2);
  p.command.height_features_per_horizon =
    cmd["height_features_per_horizon"].as<int>(0);
  p.command.height_setpoint_dim = cmd["height_setpoint_dim"].as<int>(0);
  p.command.angle_features_per_horizon = cmd["angle_features_per_horizon"].as<int>(2);
  p.command.height_lowpass_tau = cmd["height_lowpass_tau"].as<float>(0.10f);
  p.command.height_scale = cmd["height_scale"].as<float>(1.0f);

  const auto dims = doc["dims"];
  p.input_dim = dims["input_dim"].as<int>();
  p.state_dim = dims["state_dim"].as<int>();
  p.command_dim = dims["command_dim"].as<int>();
  p.output_dim = dims["output_dim"].as<int>(39);

  if (doc["model"]) {
    p.model.type = doc["model"]["type"].as<std::string>("");
    p.model.onnx_file = doc["model"]["onnx_file"].as<std::string>("generator.onnx");
    p.model.onnx_input_name = doc["model"]["onnx_input_name"].as<std::string>("obs");
    p.model.onnx_output_name = doc["model"]["onnx_output_name"].as<std::string>("reference");
  }

  if (doc["play_vel_ranges"]) {
    for (const auto& it : doc["play_vel_ranges"]) {
      const auto range = it.second;
      p.play_vel_ranges[it.first.as<std::string>()] = {
        range[0].as<float>(),
        range[1].as<float>(),
      };
    }
  }

  int sum_state = 0;
  for (const auto& name : p.state_observation_names) {
    sum_state += p.state_observations.at(name).flat_width;
  }
  if (sum_state != p.state_dim) {
    throw std::runtime_error(
      "Gen params state flat widths sum " + std::to_string(sum_state) +
      " != state_dim " + std::to_string(p.state_dim));
  }
  const int k = static_cast<int>(p.command.horizons.size());
  const int cmd_dim =
    k * (p.command.xy_features_per_horizon + p.command.angle_features_per_horizon) +
    k * p.command.height_features_per_horizon +
    p.command.height_setpoint_dim;
  if (cmd_dim != p.command_dim) {
    throw std::runtime_error(
      "Gen params command packing dim " + std::to_string(cmd_dim) +
      " != command_dim " + std::to_string(p.command_dim));
  }
  if (p.state_dim + p.command_dim != p.input_dim) {
    throw std::runtime_error("Gen params state_dim + command_dim != input_dim");
  }

  spdlog::info(
    "Loaded Gen params {} (in={} state={} cmd={} out={} hist={})",
    cfg_path.string(),
    p.input_dim,
    p.state_dim,
    p.command_dim,
    p.output_dim,
    p.history_length);
  return p;
}

std::filesystem::path resolve_generator_onnx(const GenDeployParams& params)
{
  auto path = params.params_dir / params.model.onnx_file;
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("generator ONNX missing: " + path.string());
  }
  return path;
}

}  // namespace wbc_deploy
