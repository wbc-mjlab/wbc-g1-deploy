#include "pack_arc_reference.h"

#include "DdsMotionReference.h"

#include <stdexcept>

namespace wbc_deploy {

std::vector<float> pack_arc_reference(
  const IMotionReference& ref,
  float env_origin_z)
{
  std::vector<float> arc;
  arc.reserve(static_cast<size_t>(DdsMotionReference::kArcDim));

  arc.push_back(ref.ref_base_height(env_origin_z));

  const auto lin = ref.ref_base_lin_vel_b();
  if (lin.size() != 3) {
    throw std::runtime_error("ref_base_lin_vel_b must be size 3");
  }
  arc.insert(arc.end(), lin.begin(), lin.end());

  const auto ang = ref.ref_base_ang_vel_b();
  if (ang.size() != 3) {
    throw std::runtime_error("ref_base_ang_vel_b must be size 3");
  }
  arc.insert(arc.end(), ang.begin(), ang.end());

  const auto grav = ref.ref_gravity_b();
  if (grav.size() != 3) {
    throw std::runtime_error("ref_gravity_b must be size 3");
  }
  arc.insert(arc.end(), grav.begin(), grav.end());

  const auto jp = ref.ref_joint_pos();
  if (static_cast<int>(jp.size()) != DdsMotionReference::kJointDim) {
    throw std::runtime_error(
      "ref_joint_pos must be size 29, got " + std::to_string(jp.size()));
  }
  arc.insert(arc.end(), jp.begin(), jp.end());

  if (static_cast<int>(arc.size()) != DdsMotionReference::kArcDim) {
    throw std::runtime_error("pack_arc_reference produced wrong size");
  }
  return arc;
}

}  // namespace wbc_deploy
