#include "motion_reference_obs.h"

#include "WbcMotionLoader.h"

#include <spdlog/spdlog.h>

#include <array>
#include <stdexcept>
#include <unordered_map>

namespace wbc_deploy {
namespace {

constexpr std::array<float, 3> kGravityW = {0.0f, 0.0f, -1.0f};

std::array<float, 3> quatApplyInverse(
  const Eigen::Quaternionf& q,
  const std::array<float, 3>& v)
{
  const Eigen::Vector3f out = q.conjugate() * Eigen::Vector3f(v[0], v[1], v[2]);
  return {out.x(), out.y(), out.z()};
}

using RefFn = std::vector<float> (*)(const WbcMotionLoader&, float);

std::vector<float> ref_base_height_fn(const WbcMotionLoader& loader, float env_origin_z)
{
  return {loader.ref_base_height(env_origin_z)};
}

std::vector<float> ref_base_lin_vel_b_fn(const WbcMotionLoader& loader, float)
{
  return loader.ref_base_lin_vel_b();
}

std::vector<float> ref_base_ang_vel_b_fn(const WbcMotionLoader& loader, float)
{
  return loader.ref_base_ang_vel_b();
}

std::vector<float> ref_gravity_b_fn(const WbcMotionLoader& loader, float)
{
  return loader.ref_gravity_b();
}

std::vector<float> ref_joint_pos_fn(const WbcMotionLoader& loader, float)
{
  return loader.ref_joint_pos();
}

std::vector<float> ref_joint_vel_fn(const WbcMotionLoader& loader, float)
{
  return loader.ref_joint_vel();
}

const std::unordered_map<std::string, RefFn>& reference_fn_table()
{
  static const std::unordered_map<std::string, RefFn> table = {
    {"ref_base_height", ref_base_height_fn},
    {"ref_base_lin_vel_b", ref_base_lin_vel_b_fn},
    {"ref_base_ang_vel_b", ref_base_ang_vel_b_fn},
    {"ref_gravity_b", ref_gravity_b_fn},
    {"ref_joint_pos", ref_joint_pos_fn},
    {"ref_joint_vel", ref_joint_vel_fn},
  };
  return table;
}

}  // namespace

const std::vector<std::string>& supported_reference_observation_names()
{
  static const std::vector<std::string> names = {
    "ref_base_height",
    "ref_base_lin_vel_b",
    "ref_base_ang_vel_b",
    "ref_gravity_b",
    "ref_joint_pos",
    "ref_joint_vel",
  };
  return names;
}

std::vector<float> motion_reference_observation(
  const WbcMotionLoader& loader,
  const std::string& name,
  float env_origin_z)
{
  const auto& table = reference_fn_table();
  const auto it = table.find(name);
  if (it == table.end()) {
    throw std::runtime_error("Unsupported motion reference observation: " + name);
  }
  return it->second(loader, env_origin_z);
}

std::vector<float> motion_reference_stack(
  const WbcMotionLoader& loader,
  float env_origin_z)
{
  std::vector<float> out;
  out.reserve(10 + static_cast<size_t>(loader.ref_joint_pos().size()));
  for (const auto& name : supported_reference_observation_names()) {
    if (name == "ref_joint_vel") {
      continue;
    }
    const auto chunk = motion_reference_observation(loader, name, env_origin_z);
    out.insert(out.end(), chunk.begin(), chunk.end());
  }
  return out;
}

bool is_reference_observation_name(const std::string& name)
{
  if (name == "command") {
    return true;
  }
  return reference_fn_table().count(name) > 0;
}

void validate_reference_observations(const std::vector<std::string>& names)
{
  for (const auto& name : names) {
    if (name == "command") {
      continue;
    }
    if (reference_fn_table().count(name) == 0) {
      throw std::runtime_error(
        "Policy config references unsupported observation term '" + name
        + "'. Update deploy motion_reference_obs or remove the term from training.");
    }
  }
}

}  // namespace wbc_deploy
