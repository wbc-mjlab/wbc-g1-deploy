#include "WbcMotionLoader.h"

#include "motion_npz.h"

#include <algorithm>
#include <array>

namespace {

constexpr std::array<float, 3> kGravityW = {0.0f, 0.0f, -1.0f};

std::array<float, 3> quatApplyInverse(
  const Eigen::Quaternionf& q,
  const std::array<float, 3>& v)
{
  const Eigen::Vector3f out = q.conjugate() * Eigen::Vector3f(v[0], v[1], v[2]);
  return {out.x(), out.y(), out.z()};
}

}  // namespace

WbcMotionLoader::WbcMotionLoader(
  const std::string& motion_file,
  const std::string& anchor_body_name,
  float step_dt)
: dt(step_dt)
{
  const MotionNpzArrays arrays = load_motion_npz(motion_file);
  anchor_body_index_ = resolve_anchor_body_index(
    arrays.has_body_names ? &arrays.body_names : nullptr,
    anchor_body_name);

  const auto& body_pos_w = arrays.body_pos_w;
  const auto& body_quat_w = arrays.body_quat_w;
  const auto& body_lin_vel_w = arrays.body_lin_vel_w;
  const auto& body_ang_vel_w = arrays.body_ang_vel_w;
  const auto& joint_pos = arrays.joint_pos;
  const auto& joint_vel = arrays.joint_vel;

  const size_t num_frames_npz = body_pos_w.shape[0];
  const int num_bodies = static_cast<int>(body_pos_w.shape[1]);
  const int num_joints = static_cast<int>(joint_pos.shape[1]);
  const size_t body_stride_pos = static_cast<size_t>(num_bodies) * 3;
  const size_t body_stride_quat = static_cast<size_t>(num_bodies) * 4;
  const int ab = std::clamp(anchor_body_index_, 0, num_bodies - 1);

  for (size_t i = 0; i < num_frames_npz; ++i) {
    const float* pos_base = body_pos_w.data<float>() + i * body_stride_pos;
    anchor_positions_.emplace_back(
      pos_base[ab * 3 + 0], pos_base[ab * 3 + 1], pos_base[ab * 3 + 2]);

    const float* quat_base = body_quat_w.data<float>() + i * body_stride_quat;
    anchor_quaternions_.emplace_back(
      quat_base[ab * 4 + 0],
      quat_base[ab * 4 + 1],
      quat_base[ab * 4 + 2],
      quat_base[ab * 4 + 3]);

    const float* lin_base = body_lin_vel_w.data<float>() + i * body_stride_pos;
    anchor_lin_vels_.emplace_back(
      lin_base[ab * 3 + 0], lin_base[ab * 3 + 1], lin_base[ab * 3 + 2]);

    const float* ang_base = body_ang_vel_w.data<float>() + i * body_stride_pos;
    anchor_ang_vels_.emplace_back(
      ang_base[ab * 3 + 0], ang_base[ab * 3 + 1], ang_base[ab * 3 + 2]);

    Eigen::VectorXf jp(num_joints);
    Eigen::VectorXf jv(num_joints);
    for (int j = 0; j < num_joints; ++j) {
      jp[j] = joint_pos.data<float>()[i * num_joints + j];
      jv[j] = joint_vel.data<float>()[i * num_joints + j];
    }
    dof_positions_.push_back(jp);
    dof_velocities_.push_back(jv);
  }

  num_frames = static_cast<int>(num_frames_npz);
  duration = num_frames * dt;
  frame = 0;
  update(0.0f);
}

void WbcMotionLoader::update(float time)
{
  const float phase = std::clamp(time, 0.0f, duration);
  const float f = phase / dt;
  frame = std::min(static_cast<int>(std::floor(f)), num_frames - 1);
}

void WbcMotionLoader::reset(
  const isaaclab::ArticulationData& data,
  float t)
{
  (void)data;
  update(t);
}

Eigen::Vector3f WbcMotionLoader::anchor_pos_w() const
{
  return anchor_positions_[frame];
}

Eigen::Quaternionf WbcMotionLoader::anchor_quat_w() const
{
  return anchor_quaternions_[frame];
}

Eigen::Vector3f WbcMotionLoader::anchor_lin_vel_w() const
{
  return anchor_lin_vels_[frame];
}

Eigen::Vector3f WbcMotionLoader::anchor_ang_vel_w() const
{
  return anchor_ang_vels_[frame];
}

Eigen::VectorXf WbcMotionLoader::joint_pos() const
{
  return dof_positions_[frame];
}

float WbcMotionLoader::ref_base_height(float env_origin_z) const
{
  return anchor_pos_w().z() - env_origin_z;
}

std::vector<float> WbcMotionLoader::ref_base_lin_vel_b() const
{
  const Eigen::Quaternionf q = anchor_quat_w();
  const std::array<float, 3> lin_w = {
    anchor_lin_vel_w().x(), anchor_lin_vel_w().y(), anchor_lin_vel_w().z()};
  const auto lin_b = quatApplyInverse(q, lin_w);
  return {lin_b[0], lin_b[1], lin_b[2]};
}

std::vector<float> WbcMotionLoader::ref_base_ang_vel_b() const
{
  const Eigen::Quaternionf q = anchor_quat_w();
  const std::array<float, 3> ang_w = {
    anchor_ang_vel_w().x(), anchor_ang_vel_w().y(), anchor_ang_vel_w().z()};
  const auto ang_b = quatApplyInverse(q, ang_w);
  return {ang_b[0], ang_b[1], ang_b[2]};
}

std::vector<float> WbcMotionLoader::ref_gravity_b() const
{
  const Eigen::Quaternionf q = anchor_quat_w();
  const auto grav_b = quatApplyInverse(q, kGravityW);
  return {grav_b[0], grav_b[1], grav_b[2]};
}

std::vector<float> WbcMotionLoader::ref_joint_pos() const
{
  const Eigen::VectorXf jp = joint_pos();
  std::vector<float> out(jp.size());
  for (int i = 0; i < jp.size(); ++i) {
    out[i] = jp[i];
  }
  return out;
}

std::vector<float> WbcMotionLoader::ref_joint_vel() const
{
  const Eigen::VectorXf jv = dof_velocities_[frame];
  std::vector<float> out(jv.size());
  for (int i = 0; i < jv.size(); ++i) {
    out[i] = jv[i];
  }
  return out;
}
