#include "DdsMotionReference.h"

#include "idl/WbcReference.hpp"

#include <unitree/common/dds/dds_entity.hpp>
#include <unitree/common/dds/dds_qos.hpp>
#include <unitree/common/dds/dds_topic_channel.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace wbc_deploy {
namespace {

using RefMsg = wbc_g1_deploy::msg::dds_::WbcReference_;

void fill_default_standing_arc(
  std::array<float, DdsMotionReference::kArcDim>& arc,
  const std::vector<float>& default_joint_pos)
{
  arc.fill(0.0f);
  arc[0] = 0.844f;   // torso height
  arc[9] = -1.0f;    // gravity_b z (standing)
  const int n = std::min(
    DdsMotionReference::kJointDim,
    static_cast<int>(default_joint_pos.size()));
  for (int i = 0; i < n; ++i) {
    arc[DdsMotionReference::kJointOffset + i] = default_joint_pos[static_cast<size_t>(i)];
  }
}

}  // namespace

std::string cyclonedds_local_config(int domain_id, const std::string& iface)
{
  return "<CycloneDDS><Domain id=\"" + std::to_string(domain_id) +
         "\"><General><Interfaces>"
         "<NetworkInterface name=\"" +
         iface +
         "\" priority=\"default\" multicast=\"false\"/>"
         "</Interfaces>"
         "<AllowMulticast>false</AllowMulticast>"
         "</General>"
         "<Discovery>"
         "<ParticipantIndex>auto</ParticipantIndex>"
         "<Peers><Peer address=\"127.0.0.1\"/></Peers>"
         "</Discovery>"
         "</Domain></CycloneDDS>";
}

void apply_cyclonedds_env(int domain_id, const std::string& iface)
{
  setenv("CYCLONEDDS_URI", cyclonedds_local_config(domain_id, iface).c_str(), 1);
}

ReferenceDdsConfig load_reference_dds_config(const YAML::Node& root, const YAML::Node& fsm_cfg)
{
  ReferenceDdsConfig out;
  YAML::Node block;
  if (fsm_cfg && fsm_cfg["reference_dds"]) {
    block = fsm_cfg["reference_dds"];
  } else if (root && root["reference_dds"]) {
    block = root["reference_dds"];
  }
  if (!block) {
    return out;
  }
  if (block["domain_id"]) {
    out.domain_id = block["domain_id"].as<int>();
  }
  if (block["interface"]) {
    out.interface = block["interface"].as<std::string>();
  }
  if (block["topic"]) {
    out.topic = block["topic"].as<std::string>();
  }
  return out;
}

DdsMotionReference::DdsMotionReference(
  std::vector<float> default_joint_pos,
  ReferenceDdsConfig dds,
  float step_dt)
: dds_(std::move(dds)), step_dt_(step_dt)
{
  if (static_cast<int>(default_joint_pos.size()) < kJointDim) {
    default_joint_pos.resize(static_cast<size_t>(kJointDim), 0.0f);
  }
  fill_default_standing_arc(arc_, default_joint_pos);
  for (int i = 0; i < kJointDim; ++i) {
    joints_[static_cast<size_t>(i)] = default_joint_pos[static_cast<size_t>(i)];
  }

  // Own participant on domain 101 (etc.) — never Unitree ChannelFactory domain 0.
  // Pass Cyclone config to the participant only; do not overwrite CYCLONEDDS_URI
  // after Unitree ChannelFactory::Init(0, ...) is already running.
  unitree::common::DdsParticipantQos participant_qos;
  participant_ = std::make_shared<unitree::common::DdsParticipant>(
    static_cast<uint32_t>(dds_.domain_id),
    participant_qos,
    cyclonedds_local_config(dds_.domain_id, dds_.interface));

  unitree::common::DdsSubscriberQos subscriber_qos;
  subscriber_ = std::make_shared<unitree::common::DdsSubscriber>(participant_, subscriber_qos);

  unitree::common::DdsTopicQos topic_qos;
  channel_ = std::make_shared<unitree::common::DdsTopicChannel<RefMsg>>();
  channel_->SetTopic(participant_, dds_.topic, topic_qos);

  unitree::common::DdsReaderQos reader_qos;
  unitree::common::DdsReaderCallback reader_cb([this](const void* data) {
    onMessage(data);
  });
  channel_->SetReader(subscriber_, reader_qos, reader_cb, 16);

  spdlog::info(
    "WBC reference DDS subscribe: domain {} on {} topic '{}' (isolated from Unitree domain 0)",
    dds_.domain_id,
    dds_.interface,
    dds_.topic);
}

DdsMotionReference::~DdsMotionReference() = default;

void DdsMotionReference::onMessage(const void* msg)
{
  const auto* ref = static_cast<const RefMsg*>(msg);
  if (ref == nullptr) {
    return;
  }
  const auto& arc = ref->arc();
  if (static_cast<int>(arc.size()) != kArcDim) {
    spdlog::warn(
      "Ignoring WBC reference DDS sample: arc size {} != {}",
      arc.size(),
      kArcDim);
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  const auto& meta = ref->meta();
  if (!meta.empty() && !meta[0].values().empty()) {
    noteEpisodeLocked(static_cast<uint64_t>(meta[0].values()[0]));
  }

  // Floor hold: keep local getup frame 0 Arc, but still watch episode + clip
  // name so ctrl can release only when getup actually starts (not idle/play).
  if (hold_arc_.load()) {
    if (!meta.empty()) {
      clip_name_ = meta[0].name();
    }
    time_frame_ = ref->step();
    mode_ = ref->mode().empty() ? "stream" : ref->mode();
    return;
  }

  for (int i = 0; i < kArcDim; ++i) {
    arc_[static_cast<size_t>(i)] = arc[static_cast<size_t>(i)];
  }
  const auto& jp = ref->joint_pos();
  if (static_cast<int>(jp.size()) == kJointDim) {
    for (int i = 0; i < kJointDim; ++i) {
      joints_[static_cast<size_t>(i)] = jp[static_cast<size_t>(i)];
    }
    joints_from_msg_ = true;
  } else {
    for (int i = 0; i < kJointDim; ++i) {
      joints_[static_cast<size_t>(i)] = arc_[static_cast<size_t>(kJointOffset + i)];
    }
    joints_from_msg_ = false;
  }
  time_frame_ = ref->step();
  mode_ = ref->mode().empty() ? "stream" : ref->mode();
  if (!meta.empty()) {
    clip_name_ = meta[0].name();
  }
  has_sample_.store(true);
}

void DdsMotionReference::noteEpisodeLocked(uint64_t episode)
{
  if (!have_episode_ || episode != episode_) {
    if (have_episode_) {
      restart_pending_ = true;
    }
    episode_ = episode;
    have_episode_ = true;
  }
}

void DdsMotionReference::applyArcCommand(
  const float* data,
  int n,
  const float* joint_pos,
  int joint_n,
  uint64_t time_frame,
  const std::string& mode,
  uint64_t episode,
  const std::string& clip_name,
  bool update_episode)
{
  if (data == nullptr || n != kArcDim) {
    throw std::invalid_argument("applyArcCommand expects 39 floats");
  }
  std::lock_guard<std::mutex> lock(mtx_);
  for (int i = 0; i < kArcDim; ++i) {
    arc_[static_cast<size_t>(i)] = data[i];
  }
  if (joint_pos != nullptr && joint_n == kJointDim) {
    for (int i = 0; i < kJointDim; ++i) {
      joints_[static_cast<size_t>(i)] = joint_pos[i];
    }
    joints_from_msg_ = true;
  } else {
    for (int i = 0; i < kJointDim; ++i) {
      joints_[static_cast<size_t>(i)] = arc_[static_cast<size_t>(kJointOffset + i)];
    }
    joints_from_msg_ = false;
  }
  time_frame_ = time_frame;
  mode_ = mode;
  if (!clip_name.empty()) {
    clip_name_ = clip_name;
  }
  if (update_episode) {
    noteEpisodeLocked(episode);
  }
  has_sample_.store(true);
}

void DdsMotionReference::update(float /*time*/)
{
  // Latest sample is applied asynchronously in onMessage.
}

void DdsMotionReference::reset(const isaaclab::ArticulationData& /*data*/, float /*t*/)
{
  // Keep last / default arc; publisher owns timing.
}

std::array<float, DdsMotionReference::kArcDim> DdsMotionReference::copyArc() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return arc_;
}

std::array<float, DdsMotionReference::kJointDim> DdsMotionReference::copyJoints() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return joints_;
}

Eigen::VectorXf DdsMotionReference::joint_pos() const
{
  const auto j = copyJoints();
  Eigen::VectorXf out(kJointDim);
  for (int i = 0; i < kJointDim; ++i) {
    out[i] = j[static_cast<size_t>(i)];
  }
  return out;
}

float DdsMotionReference::ref_base_height(float /*env_origin_z*/) const
{
  return copyArc()[0];
}

std::vector<float> DdsMotionReference::ref_base_lin_vel_b() const
{
  const auto a = copyArc();
  return {a[1], a[2], a[3]};
}

std::vector<float> DdsMotionReference::ref_base_ang_vel_b() const
{
  const auto a = copyArc();
  return {a[4], a[5], a[6]};
}

std::vector<float> DdsMotionReference::ref_gravity_b() const
{
  const auto a = copyArc();
  return {a[7], a[8], a[9]};
}

std::vector<float> DdsMotionReference::ref_joint_pos() const
{
  const auto j = copyJoints();
  return std::vector<float>(j.begin(), j.end());
}

std::vector<float> DdsMotionReference::ref_joint_vel() const
{
  return std::vector<float>(static_cast<size_t>(kJointDim), 0.0f);
}

float DdsMotionReference::duration() const
{
  // Streaming / Gen: treat as open-ended.
  return 1.0e6f;
}

int DdsMotionReference::frame() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(time_frame_);
}

std::string DdsMotionReference::mode() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return mode_;
}

std::string DdsMotionReference::clip_name() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return clip_name_;
}

uint64_t DdsMotionReference::episode() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return episode_;
}

bool DdsMotionReference::consume_restart()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!restart_pending_) {
    return false;
  }
  restart_pending_ = false;
  return true;
}

}  // namespace wbc_deploy
