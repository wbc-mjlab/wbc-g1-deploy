#include "wbc_tracking_params.h"

#include <spdlog/spdlog.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace wbc_deploy {
namespace {

constexpr const char* kTrainingParams = "wbc_tracking_params.yaml";
constexpr const char* kDeployParams = "deploy.yaml";

YAML::Node load_yaml_file(const std::filesystem::path& path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Failed to open YAML: " + path.string());
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  std::string text = ss.str();
  if (!text.empty() && text[0] == '#') {
    const auto pos = text.find('\n');
    text = (pos == std::string::npos) ? std::string() : text.substr(pos + 1);
  }
  return YAML::Load(text);
}

YAML::Node load_robot_defaults(const std::filesystem::path& policy_dir, const std::string& robot_id)
{
  const auto defaults_path = policy_dir.parent_path().parent_path() / "policy_defaults.yaml";
  if (std::filesystem::exists(defaults_path)) {
    return load_yaml_file(defaults_path);
  }
  (void)robot_id;
  return YAML::Node(YAML::NodeType::Map);
}

std::string resolve_action_mode(const YAML::Node& doc)
{
  if (doc["action"] && doc["action"]["action_mode"]) {
    return doc["action"]["action_mode"].as<std::string>();
  }
  if (doc["tracking"] && doc["tracking"]["action_mode"]) {
    return doc["tracking"]["action_mode"].as<std::string>();
  }
  const std::string legacy_type = doc["action"] ? doc["action"]["type"].as<std::string>("") : "";
  if (legacy_type == "ReferenceJointPositionAction") {
    return "reference_residual";
  }
  if (legacy_type == "JointPositionAction") {
    return "default_relative";
  }
  return "reference_residual";
}

std::string deploy_action_class(const std::string& action_mode)
{
  if (action_mode == "reference_residual") {
    return "ReferenceJointPositionAction";
  }
  if (action_mode == "default_relative") {
    return "JointPositionAction";
  }
  throw std::runtime_error("Unsupported action_mode: " + action_mode);
}

int resolve_actor_history_length(const YAML::Node& doc, const YAML::Node& defaults)
{
  if (doc["tracking"] && doc["tracking"]["actor_history_length"]) {
    return doc["tracking"]["actor_history_length"].as<int>();
  }
  if (defaults["wbc_tracking"] && defaults["wbc_tracking"]["actor_history_length"]) {
    return defaults["wbc_tracking"]["actor_history_length"].as<int>();
  }
  if (defaults["tracking"] && defaults["tracking"]["actor_history_length"]) {
    return defaults["tracking"]["actor_history_length"].as<int>();
  }
  return 1;
}

std::string observation_deploy_name(const std::string& training_name)
{
  if (training_name == "joint_pos") {
    return "joint_pos_rel";
  }
  if (training_name == "joint_vel") {
    return "joint_vel_rel";
  }
  if (training_name == "actions") {
    return "last_action";
  }
  return training_name;
}

YAML::Node tracking_params_to_deploy(const YAML::Node& doc, const YAML::Node& defaults)
{
  const std::string robot_id = doc["robot_id"]
    ? doc["robot_id"].as<std::string>()
    : defaults["robot_id"].as<std::string>("g1");
  const int joint_count = static_cast<int>(doc["joint_names"].size());
  const std::string action_mode = resolve_action_mode(doc);
  const std::string action_class = deploy_action_class(action_mode);
  const int history_length = resolve_actor_history_length(doc, defaults);

  YAML::Node observations(YAML::NodeType::Map);
  for (const auto& entry : doc["actor_observations"]) {
    const std::string train_name = entry.first.as<std::string>();
    const YAML::Node block = entry.second;
    const std::string deploy_name = observation_deploy_name(train_name);

    YAML::Node obs;
    obs["params"] = block["params"] ? block["params"] : YAML::Node(YAML::NodeType::Map);
    obs["clip"] = YAML::Null;
    obs["scale"] = block["scale"];
    obs["history_length"] = history_length;
    observations[deploy_name] = obs;
  }

  YAML::Node action;
  if (doc["action"]["clip"]) {
    action["clip"] = doc["action"]["clip"];
  } else {
    action["clip"] = YAML::Node(YAML::NodeType::Null);
  }
  action["joint_names"] = doc["action"]["joint_names"]
    ? doc["action"]["joint_names"]
    : YAML::Load("[.*]");
  action["scale"] = doc["action"]["scale"];
  if (doc["action"]["offset"]) {
    action["offset"] = doc["action"]["offset"];
  } else {
    action["offset"] = YAML::Node(YAML::NodeType::Null);
  }
  if (doc["action"]["joint_ids"]) {
    action["joint_ids"] = doc["action"]["joint_ids"];
  } else {
    action["joint_ids"] = YAML::Node(YAML::NodeType::Null);
  }
  action["command_name"] = doc["action"]["command_name"]
    ? doc["action"]["command_name"].as<std::string>()
    : "motion";

  YAML::Node wbc_tracking(YAML::NodeType::Map);
  if (defaults["wbc_tracking"]) {
    for (auto it = defaults["wbc_tracking"].begin(); it != defaults["wbc_tracking"].end(); ++it) {
      wbc_tracking[it->first] = it->second;
    }
  } else if (defaults["tracking"]) {
    for (auto it = defaults["tracking"].begin(); it != defaults["tracking"].end(); ++it) {
      wbc_tracking[it->first] = it->second;
    }
  }
  if (doc["tracking"]) {
    for (auto it = doc["tracking"].begin(); it != doc["tracking"].end(); ++it) {
      wbc_tracking[it->first] = it->second;
    }
  }
  wbc_tracking["robot_id"] = robot_id;
  wbc_tracking["action_mode"] = action_mode;
  wbc_tracking["actor_history_length"] = history_length;
  if (doc["tracking"]["actor_observation_names"]) {
    wbc_tracking["training_observation_names"] = doc["tracking"]["actor_observation_names"];
  }
  YAML::Node deploy_obs_names(YAML::NodeType::Sequence);
  for (const auto& entry : observations) {
    deploy_obs_names.push_back(entry.first.as<std::string>());
  }
  wbc_tracking["deploy_observation_names"] = deploy_obs_names;

  YAML::Node deploy(YAML::NodeType::Map);
  if (defaults["joint_ids_map"]) {
    deploy["joint_ids_map"] = defaults["joint_ids_map"];
  } else {
    YAML::Node ids(YAML::NodeType::Sequence);
    for (int i = 0; i < joint_count; ++i) {
      ids.push_back(i);
    }
    deploy["joint_ids_map"] = ids;
  }
  deploy["step_dt"] = doc["policy_step_dt"];
  deploy["stiffness"] = defaults["stiffness"] ? defaults["stiffness"] : doc["stiffness"];
  deploy["damping"] = defaults["damping"] ? defaults["damping"] : doc["damping"];
  deploy["default_joint_pos"] = defaults["default_joint_pos"]
    ? defaults["default_joint_pos"]
    : doc["default_joint_pos"];
  deploy["commands"] = YAML::Node(YAML::NodeType::Map);
  deploy["actions"] = YAML::Node(YAML::NodeType::Map);
  deploy["actions"][action_class] = action;
  deploy["observations"] = observations;
  deploy["wbc_tracking"] = wbc_tracking;
  return deploy;
}

}  // namespace

YAML::Node load_policy_config(const std::filesystem::path& policy_dir)
{
  const auto training_path = policy_dir / "params" / kTrainingParams;
  const auto deploy_path = policy_dir / "params" / kDeployParams;

  if (std::filesystem::exists(training_path)) {
    const YAML::Node training = load_yaml_file(training_path);
    const std::string robot_id = training["robot_id"]
      ? training["robot_id"].as<std::string>()
      : "g1";
    const YAML::Node defaults = load_robot_defaults(policy_dir, robot_id);
    spdlog::info("Loaded policy config from {}", training_path.string());
    return tracking_params_to_deploy(training, defaults);
  }

  if (std::filesystem::exists(deploy_path)) {
    spdlog::info("Loaded policy config from {}", deploy_path.string());
    return load_yaml_file(deploy_path);
  }

  throw std::runtime_error(
    "No policy config in " + policy_dir.string()
    + " (expected params/" + kTrainingParams + " or params/" + kDeployParams + ")");
}

std::filesystem::path resolve_onnx_path(const std::filesystem::path& policy_dir)
{
  const std::vector<std::filesystem::path> candidates = {
    policy_dir / "params" / "latest.onnx",
    policy_dir / "exported" / "policy.onnx",
    policy_dir / "params" / "policy.onnx",
  };
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      spdlog::info("ONNX model: {}", path.string());
      return path;
    }
  }
  throw std::runtime_error("No ONNX model found under " + policy_dir.string());
}

}  // namespace wbc_deploy
