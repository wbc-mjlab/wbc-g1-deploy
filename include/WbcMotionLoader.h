#pragma once

#include "isaaclab/assets/articulation/articulation.h"
#include <Eigen/Dense>

#include <string>
#include <vector>


class WbcMotionLoader
{
public:
  WbcMotionLoader(
    const std::string& motion_file,
    const std::string& anchor_body_name,
    float step_dt);

  void update(float time);
  void reset(const isaaclab::ArticulationData& data, float t = 0.0f);

  Eigen::Vector3f anchor_pos_w() const;
  Eigen::Quaternionf anchor_quat_w() const;
  Eigen::Vector3f anchor_lin_vel_w() const;
  Eigen::Vector3f anchor_ang_vel_w() const;
  Eigen::VectorXf joint_pos() const;
  float ref_base_height(float env_origin_z) const;
  std::vector<float> ref_base_lin_vel_b() const;
  std::vector<float> ref_base_ang_vel_b() const;
  std::vector<float> ref_gravity_b() const;
  std::vector<float> ref_joint_pos() const;
  std::vector<float> ref_joint_vel() const;

  float dt;
  int num_frames;
  float duration;
  int frame;

private:
  int anchor_body_index_ = 0;
  std::vector<Eigen::Vector3f> anchor_positions_;
  std::vector<Eigen::Quaternionf> anchor_quaternions_;
  std::vector<Eigen::Vector3f> anchor_lin_vels_;
  std::vector<Eigen::Vector3f> anchor_ang_vels_;
  std::vector<Eigen::VectorXf> dof_positions_;
  std::vector<Eigen::VectorXf> dof_velocities_;
};
