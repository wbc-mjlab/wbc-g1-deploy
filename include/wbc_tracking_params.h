#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace wbc_deploy {

/// Load runtime deploy config from wbc_mjlab ``config.yaml`` (or legacy deploy.yaml).
YAML::Node load_policy_config(const std::filesystem::path& policy_dir);

/// Resolve ONNX model path (``params/policy.onnx``, legacy ``params/latest.onnx``).
std::filesystem::path resolve_onnx_path(const std::filesystem::path& policy_dir);

}  // namespace wbc_deploy
