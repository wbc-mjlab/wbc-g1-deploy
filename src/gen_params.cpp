#include "gen_params.h"

#include <spdlog/spdlog.h>

#include <algorithm>
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

GenCommandTermCfg parse_command_term(const std::string& name, const YAML::Node& term)
{
  GenCommandTermCfg cfg;
  cfg.deploy_name = term["deploy_name"].as<std::string>(name);
  cfg.training_name = term["training_name"].as<std::string>(name);
  cfg.dim = term["dim"].as<int>();
  cfg.history_length = term["history_length"].as<int>(0);
  cfg.flat_width = term["flat_width"].as<int>(cfg.dim);
  cfg.scale = as_float_list(term["scale"], cfg.dim);
  if (term["horizons"]) {
    for (const auto& h : term["horizons"]) {
      cfg.horizons.push_back(h.as<int>());
    }
  }
  cfg.position_scale = term["position_scale"].as<float>(1.0f);
  cfg.height_scale = term["height_scale"].as<float>(1.0f);
  cfg.height_lowpass_tau = term["height_lowpass_tau"].as<float>(0.0f);
  cfg.height_setpoint_dim = term["height_setpoint_dim"].as<int>(0);
  cfg.features_per_horizon = term["features_per_horizon"].as<int>(0);
  return cfg;
}

void parse_modular_command(const YAML::Node& cmd, GenCommandCfg& out)
{
  for (const auto& n : cmd["term_names"]) {
    out.term_names.push_back(n.as<std::string>());
  }
  if (out.term_names.empty()) {
    throw std::runtime_error("Gen params command missing term_names");
  }
  const auto obs = cmd["observations"];
  if (!obs) {
    throw std::runtime_error(
      "Gen params command missing observations (v2 modular layout required)");
  }
  for (const auto& name : out.term_names) {
    if (!obs[name]) {
      throw std::runtime_error("Gen params missing command.observations." + name);
    }
    out.observations[name] = parse_command_term(name, obs[name]);
  }
  if (cmd["horizons"]) {
    for (const auto& h : cmd["horizons"]) {
      out.horizons.push_back(h.as<int>());
    }
  }
  const auto xy_it = out.observations.find("cmd_xy_waypoints");
  const auto hset_it = out.observations.find("cmd_height_setpoint");
  const auto hwp_it = out.observations.find("cmd_height_waypoints");
  if (out.horizons.empty() && xy_it != out.observations.end()) {
    out.horizons = xy_it->second.horizons;
  }
  if (out.horizons.empty() && hwp_it != out.observations.end()) {
    out.horizons = hwp_it->second.horizons;
  }
  out.position_scale =
    xy_it != out.observations.end() ? xy_it->second.position_scale : 1.0f;
  if (hset_it != out.observations.end()) {
    out.height_scale = hset_it->second.height_scale;
    out.height_setpoint_dim = hset_it->second.flat_width;
  } else if (hwp_it != out.observations.end()) {
    out.height_scale = hwp_it->second.height_scale;
    out.height_lowpass_tau = hwp_it->second.height_lowpass_tau;
    out.height_features_per_horizon =
      hwp_it->second.features_per_horizon > 0 ? hwp_it->second.features_per_horizon
                                              : 1;
  }
}

bool is_known_gen_model_type(const std::string& type)
{
  static const char* kKnown[] = {
    "mlp", "flow", "flow_ae", "cascade_ae", "gru", "vae",
  };
  for (const char* t : kKnown) {
    if (type == t) {
      return true;
    }
  }
  return false;
}

void parse_model(const YAML::Node& model, GenModelCfg& out)
{
  out.type = model["type"].as<std::string>("");
  out.onnx_file = model["onnx_file"].as<std::string>("generator.onnx");
  out.onnx_input_name = model["onnx_input_name"].as<std::string>("obs");
  out.onnx_output_name = model["onnx_output_name"].as<std::string>("reference");
  out.n_euler_steps = model["n_euler_steps"].as<int>(0);
  out.latent_dim = model["latent_dim"].as<int>(0);
  out.intent_dim = model["intent_dim"].as<int>(0);
  out.refine = model["refine"].as<bool>(false);
  out.use_ae = model["use_ae"].as<bool>(false);
  out.history_length = model["history_length"].as<int>(0);

  if (out.type.empty()) {
    throw std::runtime_error("Gen params model.type is required");
  }
  if (!is_known_gen_model_type(out.type)) {
    throw std::runtime_error(
      "Unsupported Gen model.type '" + out.type +
      "' (expected mlp|flow|flow_ae|cascade_ae|gru|vae)");
  }
  // Inference is always ONNX ``sample`` / ``forward``; metadata is for logging
  // and sanity checks only.
  if (out.type == "flow" || out.type == "flow_ae") {
    if (out.n_euler_steps <= 0) {
      spdlog::warn(
        "Gen model.type={} missing n_euler_steps in config (ONNX still runs)",
        out.type);
    }
  }
  if (out.type == "cascade_ae") {
    if (out.intent_dim <= 0) {
      spdlog::warn(
        "Gen model.type=cascade_ae missing intent_dim in config (ONNX still runs)");
    }
  }
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
  if (p.schema_version != "wbc_gen_deploy_params_v2") {
    throw std::runtime_error(
      "Expected schema_version wbc_gen_deploy_params_v2, got '" +
      p.schema_version + "'");
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
  parse_modular_command(cmd, p.command);

  const auto dims = doc["dims"];
  p.input_dim = dims["input_dim"].as<int>();
  p.state_dim = dims["state_dim"].as<int>();
  p.command_dim = dims["command_dim"].as<int>();
  p.output_dim = dims["output_dim"].as<int>(39);

  if (!doc["model"]) {
    throw std::runtime_error("Gen params missing 'model' block");
  }
  parse_model(doc["model"], p.model);

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
  int cmd_dim = 0;
  for (const auto& name : p.command.term_names) {
    cmd_dim += p.command.observations.at(name).flat_width;
  }
  if (cmd_dim != p.command_dim) {
    throw std::runtime_error(
      "Gen params command flat widths sum " + std::to_string(cmd_dim) +
      " != command_dim " + std::to_string(p.command_dim));
  }
  if (p.state_dim + p.command_dim != p.input_dim) {
    throw std::runtime_error("Gen params state_dim + command_dim != input_dim");
  }

  spdlog::info(
    "Loaded Gen params {} (type={} in={} state={} cmd={} out={} hist={})",
    cfg_path.string(),
    p.model.type,
    p.input_dim,
    p.state_dim,
    p.command_dim,
    p.output_dim,
    p.history_length);
  if (p.model.type == "cascade_ae") {
    spdlog::info(
      "  cascade_ae meta: intent_dim={} refine={} use_ae={} latent_dim={}",
      p.model.intent_dim,
      p.model.refine,
      p.model.use_ae,
      p.model.latent_dim);
  } else if (p.model.type == "flow_ae" || p.model.type == "flow") {
    spdlog::info(
      "  {} meta: n_euler_steps={} latent_dim={}",
      p.model.type,
      p.model.n_euler_steps,
      p.model.latent_dim);
  }
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
