#pragma once

#include "IMotionReference.h"

#include <string>
#include <vector>


class WbcMotionLoader : public IMotionReference
{
public:
  WbcMotionLoader(
    const std::string& motion_file,
    const std::string& anchor_body_name,
    float step_dt);

  void update(float time) override;
  void reset(const isaaclab::ArticulationData& data, float t = 0.0f) override;

  Eigen::Vector3f anchor_pos_w() const;
  Eigen::Quaternionf anchor_quat_w() const;
  Eigen::Vector3f anchor_lin_vel_w() const;
  Eigen::Vector3f anchor_ang_vel_w() const;
  Eigen::VectorXf joint_pos() const override;
  float ref_base_height(float env_origin_z) const override;
  std::vector<float> ref_base_lin_vel_b() const override;
  std::vector<float> ref_base_ang_vel_b() const override;
  std::vector<float> ref_gravity_b() const override;
  std::vector<float> ref_joint_pos() const override;
  std::vector<float> ref_joint_vel() const override;

  float duration() const override { return duration_; }
  int frame() const override { return frame_; }

  float dt = 0.02f;
  int num_frames = 0;

private:
  float duration_ = 0.0f;
  int frame_ = 0;
  int anchor_body_index_ = 0;
  std::vector<Eigen::Vector3f> anchor_positions_;
  std::vector<Eigen::Quaternionf> anchor_quaternions_;
  std::vector<Eigen::Vector3f> anchor_lin_vels_;
  std::vector<Eigen::Vector3f> anchor_ang_vels_;
  std::vector<Eigen::VectorXf> dof_positions_;
  std::vector<Eigen::VectorXf> dof_velocities_;
};
