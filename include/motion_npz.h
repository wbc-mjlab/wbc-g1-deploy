#pragma once

#include "cnpy.h"

#include <string>
#include <vector>
#include <filesystem>

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

/// Joint positions for one NPZ frame (deploy joint order).
std::vector<float> load_motion_joint_pos_at(const std::string& path, size_t frame_index);

/// Resolve a clip name or relative path under clips_dir.
std::filesystem::path resolve_clip_path(
  const std::filesystem::path& clips_dir,
  const std::string& clip_name);

/// Resolve anchor body index from optional body_names array or G1 fallback list.
int resolve_anchor_body_index(
  const cnpy::NpyArray* body_names,
  const std::string& anchor_name);
