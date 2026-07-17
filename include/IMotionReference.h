#pragma once

#include "isaaclab/assets/articulation/articulation.h"

#include <Eigen/Dense>

#include <string>
#include <vector>


/// Runtime reference source for WBC actor ``ref_*`` terms + residual ``q_ref``.
///
/// Local NPZ clips (``WbcMotionLoader``) and DDS samples (``DdsMotionReference``)
/// both implement this so MDP / actions stay source-agnostic.
class IMotionReference
{
public:
  virtual ~IMotionReference() = default;

  virtual void update(float time) = 0;
  virtual void reset(const isaaclab::ArticulationData& data, float t = 0.0f) = 0;

  virtual Eigen::VectorXf joint_pos() const = 0;
  virtual float ref_base_height(float env_origin_z) const = 0;
  virtual std::vector<float> ref_base_lin_vel_b() const = 0;
  virtual std::vector<float> ref_base_ang_vel_b() const = 0;
  virtual std::vector<float> ref_gravity_b() const = 0;
  virtual std::vector<float> ref_joint_pos() const = 0;
  virtual std::vector<float> ref_joint_vel() const = 0;

  virtual float duration() const = 0;
  virtual int frame() const = 0;

  /// True once when the reference publisher starts a new clip / Gen episode.
  /// Ctrl should ``env->reset()`` so policy history stays in sync.
  virtual bool consume_restart() { return false; }
};
