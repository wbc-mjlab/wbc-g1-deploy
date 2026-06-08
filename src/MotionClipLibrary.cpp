#include "MotionClipLibrary.h"
#include "State_WbcTracking.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>


MotionClipLibrary::~MotionClipLibrary() = default;

MotionClipLibrary::MotionClipLibrary(
  std::filesystem::path clips_dir,
  std::filesystem::path manifest_path,
  float step_dt,
  const std::string& anchor_body_name)
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
}

const std::string& MotionClipLibrary::currentName() const
{
  return clips_.at(static_cast<size_t>(current_index_)).name;
}

bool MotionClipLibrary::loadClipAt(int index)
{
  if (index < 0 || index >= static_cast<int>(clips_.size())) {
    return false;
  }
  current_index_ = index;
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

bool MotionClipLibrary::selectClip(int index)
{
  return loadClipAt(index);
}

bool MotionClipLibrary::nextClip()
{
  const int next = (current_index_ + 1) % static_cast<int>(clips_.size());
  return loadClipAt(next);
}

bool MotionClipLibrary::prevClip()
{
  const int n = static_cast<int>(clips_.size());
  const int prev = (current_index_ - 1 + n) % n;
  return loadClipAt(prev);
}

void MotionClipLibrary::resetPlayback(
  const isaaclab::ArticulationData& data,
  float time_start)
{
  if (loader_) {
    loader_->reset(data, time_start);
  }
}
