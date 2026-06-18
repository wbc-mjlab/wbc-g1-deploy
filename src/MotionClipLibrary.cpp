#include "MotionClipLibrary.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "motion_npz.h"

#include <stdexcept>


MotionClipLibrary::~MotionClipLibrary() = default;

MotionClipLibrary::MotionClipLibrary(
  std::filesystem::path clips_dir,
  std::filesystem::path manifest_path,
  float step_dt,
  const std::string& anchor_body_name,
  const std::unordered_map<std::string, std::string>& pose_clips)
: clips_dir_(std::move(clips_dir))
, step_dt_(step_dt)
, anchor_body_(anchor_body_name)
{
  if (!std::filesystem::exists(manifest_path)) {
    throw std::runtime_error("clips manifest not found: " + manifest_path.string());
  }

  const YAML::Node root = YAML::LoadFile(manifest_path.string());
  if (!root["clips"]) {
    throw std::runtime_error("manifest missing 'clips' list");
  }

  for (const auto& entry : root["clips"]) {
    ClipEntry clip;
    clip.name = entry["name"].as<std::string>();
    const auto rel = entry["file"].as<std::string>();
    clip.path = clips_dir_ / rel;
    if (!std::filesystem::exists(clip.path)) {
      throw std::runtime_error("clip file missing: " + clip.path.string());
    }
    clips_.push_back(std::move(clip));
  }

  if (clips_.empty()) {
    throw std::runtime_error("manifest has no clips");
  }

  for (const auto& [key, clip_name] : pose_clips) {
    ClipEntry clip;
    clip.name = clip_name;
    clip.path = resolve_clip_path(clips_dir_, clip_name);
    if (!std::filesystem::exists(clip.path)) {
      throw std::runtime_error(
        "pose clip missing for '" + key + "': " + clip.path.string());
    }
    pose_clips_[key] = std::move(clip);
    spdlog::info("Registered pose clip {} -> {}", key, clip_name);
  }

  int start_index = 0;
  if (root["default"]) {
    const std::string def = root["default"].as<std::string>();
    if (def.find_first_not_of("0123456789") == std::string::npos) {
      start_index = std::stoi(def);
    } else {
      for (size_t i = 0; i < clips_.size(); ++i) {
        if (clips_[i].name == def) {
          start_index = static_cast<int>(i);
          break;
        }
      }
    }
  }

  if (!loadClipAt(start_index)) {
    throw std::runtime_error("failed to load default clip");
  }
  selected_browsable_index_ = start_index;
}

const std::string& MotionClipLibrary::selectedBrowsableName() const
{
  return clips_.at(static_cast<size_t>(selected_browsable_index_)).name;
}

const std::string& MotionClipLibrary::currentName() const
{
  if (current_kind_ != ClipKind::Browsable) {
    const auto it = pose_clips_.find(current_pose_key_);
    if (it != pose_clips_.end()) {
      return it->second.name;
    }
  }
  return clips_.at(static_cast<size_t>(current_index_)).name;
}

bool MotionClipLibrary::loadClipAt(int index)
{
  if (index < 0 || index >= static_cast<int>(clips_.size())) {
    return false;
  }
  current_index_ = index;
  last_browsable_index_ = index;
  selected_browsable_index_ = index;
  current_kind_ = ClipKind::Browsable;
  current_pose_key_.clear();
  const auto& clip = clips_[static_cast<size_t>(current_index_)];
  loader_ = std::make_shared<WbcMotionLoader>(
    clip.path.string(), anchor_body_, step_dt_);
  spdlog::info(
    "Motion clip [{}/{}] {} ({:.2f}s)",
    current_index_ + 1,
    clips_.size(),
    clip.name,
    loader_->duration);
  return true;
}

bool MotionClipLibrary::browseNextSelected()
{
  const int n = static_cast<int>(clips_.size());
  selected_browsable_index_ = (selected_browsable_index_ + 1) % n;
  spdlog::info(
    "Selected clip [{}/{}] {}",
    selected_browsable_index_ + 1,
    n,
    selectedBrowsableName());
  return true;
}

bool MotionClipLibrary::browsePrevSelected()
{
  const int n = static_cast<int>(clips_.size());
  selected_browsable_index_ = (selected_browsable_index_ - 1 + n) % n;
  spdlog::info(
    "Selected clip [{}/{}] {}",
    selected_browsable_index_ + 1,
    n,
    selectedBrowsableName());
  return true;
}

bool MotionClipLibrary::activateSelectedBrowsable()
{
  return loadClipAt(selected_browsable_index_);
}

bool MotionClipLibrary::selectPoseClip(const std::string& key)
{
  const auto it = pose_clips_.find(key);
  if (it == pose_clips_.end()) {
    return false;
  }

  if (current_kind_ == ClipKind::Browsable) {
    last_browsable_index_ = current_index_;
  }

  current_pose_key_ = key;
  if (key == "getup") {
    current_kind_ = ClipKind::PoseGetup;
  } else if (key == "liedown") {
    current_kind_ = ClipKind::PoseLiedown;
  } else {
    current_kind_ = ClipKind::Browsable;
  }

  loader_ = std::make_shared<WbcMotionLoader>(
    it->second.path.string(), anchor_body_, step_dt_);
  spdlog::info(
    "Pose clip [{}] {} ({:.2f}s)",
    key,
    it->second.name,
    loader_->duration);
  return true;
}

void MotionClipLibrary::resetPlayback(
  const isaaclab::ArticulationData& data,
  float time_start)
{
  if (loader_) {
    loader_->reset(data, time_start);
  }
}
