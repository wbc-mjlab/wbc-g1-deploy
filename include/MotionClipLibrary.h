#pragma once

#include "WbcMotionLoader.h"
#include "isaaclab/assets/articulation/articulation.h"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
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

  enum class ClipKind
  {
    Browsable,
    PoseGetup,
    PoseLiedown,
  };

  ~MotionClipLibrary();

  MotionClipLibrary(
    std::filesystem::path clips_dir,
    std::filesystem::path manifest_path,
    float step_dt,
    const std::string& anchor_body_name,
    const std::unordered_map<std::string, std::string>& pose_clips = {});

  std::shared_ptr<WbcMotionLoader> loader() const { return loader_; }

  const std::vector<ClipEntry>& clips() const { return clips_; }
  int currentIndex() const { return current_index_; }
  int selectedBrowsableIndex() const { return selected_browsable_index_; }
  ClipKind currentKind() const { return current_kind_; }
  const std::string& currentName() const;
  const std::string& selectedBrowsableName() const;

  bool browseNextSelected();
  bool browsePrevSelected();
  bool activateSelectedBrowsable();
  bool selectBrowsableByName(const std::string& name);
  bool selectPoseClip(const std::string& key);

  void resetPlayback(const isaaclab::ArticulationData& data, float time_start);

private:
  bool loadClipAt(int index);

  std::filesystem::path clips_dir_;
  std::vector<ClipEntry> clips_;
  std::unordered_map<std::string, ClipEntry> pose_clips_;
  int current_index_ = 0;
  int selected_browsable_index_ = 0;
  int last_browsable_index_ = 0;
  ClipKind current_kind_ = ClipKind::Browsable;
  std::string current_pose_key_;
  float step_dt_ = 0.02f;
  std::string anchor_body_;
  std::shared_ptr<WbcMotionLoader> loader_;
};
