#pragma once

#include "cnpy.h"

#include <string>
#include <vector>

/// Load numeric motion arrays from a wbc_mjlab NPZ (skips object/pickled fields).
struct MotionNpzArrays {
  cnpy::NpyArray joint_pos;
  cnpy::NpyArray joint_vel;
  cnpy::NpyArray body_pos_w;
  cnpy::NpyArray body_quat_w;
  cnpy::NpyArray body_lin_vel_w;
  cnpy::NpyArray body_ang_vel_w;
  bool has_body_names = false;
  cnpy::NpyArray body_names;
};

MotionNpzArrays load_motion_npz(const std::string& path);

/// Resolve anchor body index from optional body_names array or G1 fallback list.
int resolve_anchor_body_index(
  const cnpy::NpyArray* body_names,
  const std::string& anchor_name);
