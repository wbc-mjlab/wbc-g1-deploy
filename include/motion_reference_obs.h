#pragma once

#include <string>
#include <vector>

class IMotionReference;

namespace wbc_deploy {

/// Names supported by ``motion_reference_observation`` / deploy obs handlers.
const std::vector<std::string>& supported_reference_observation_names();

/// Compute one modular motion-reference observation term from the active source.
std::vector<float> motion_reference_observation(
  const IMotionReference& ref,
  const std::string& name,
  float env_origin_z);

/// Legacy stacked reference vector (default term order, no joint velocity).
std::vector<float> motion_reference_stack(
  const IMotionReference& ref,
  float env_origin_z);

bool is_reference_observation_name(const std::string& name);

void validate_reference_observations(const std::vector<std::string>& names);

}  // namespace wbc_deploy
