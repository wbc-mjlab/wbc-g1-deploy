#pragma once

#include "WbcMotionLoader.h"
#include "isaaclab/assets/articulation/articulation.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

/// Loads multiple NPZ clips; one shared WBC policy, switchable at runtime.
class MotionClipLibrary
{
public:
  struct ClipEntry
  {
    std::string name;
    std::filesystem::path path;
  };

  ~MotionClipLibrary();

  MotionClipLibrary(
    std::filesystem::path clips_dir,
    std::filesystem::path manifest_path,
    float step_dt,
    const std::string& anchor_body_name);

  std::shared_ptr<WbcMotionLoader> loader() const { return loader_; }

  const std::vector<ClipEntry>& clips() const { return clips_; }
  int currentIndex() const { return current_index_; }
  const std::string& currentName() const;

  bool selectClip(int index);
  bool nextClip();
  bool prevClip();

  void resetPlayback(const isaaclab::ArticulationData& data, float time_start);

private:
  bool loadClipAt(int index);

  std::filesystem::path clips_dir_;
  std::vector<ClipEntry> clips_;
  int current_index_ = 0;
  float step_dt_ = 0.02f;
  std::string anchor_body_;
  std::shared_ptr<WbcMotionLoader> loader_;
};
