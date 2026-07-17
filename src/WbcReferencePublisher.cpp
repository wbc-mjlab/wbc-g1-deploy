#include "WbcReferencePublisher.h"

#include "idl/WbcReference.hpp"

#include <unitree/common/dds/dds_entity.hpp>
#include <unitree/common/dds/dds_qos.hpp>
#include <unitree/common/dds/dds_topic_channel.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <stdexcept>

namespace wbc_deploy {
namespace {

using RefMsg = wbc_g1_deploy::msg::dds_::WbcReference_;

uint64_t now_us()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

}  // namespace

std::vector<float> g1_standing_arc()
{
  return {
    0.844f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, -1.0f,
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.35f, 0.18f, 0.0f, 0.87f, 0.0f, 0.0f, 0.0f,
    0.35f, -0.18f, 0.0f, 0.87f, 0.0f, 0.0f, 0.0f,
  };
}

WbcReferencePublisher::WbcReferencePublisher(ReferenceDdsConfig dds)
: dds_(std::move(dds))
{
  // Isolated participant — never Unitree ChannelFactory domain 0.
  // Pass Cyclone config to the participant only; do not setenv CYCLONEDDS_URI
  // (that would break Unitree domain 0 if the ref node also reads lowstate).
  unitree::common::DdsParticipantQos participant_qos;
  participant_ = std::make_shared<unitree::common::DdsParticipant>(
    static_cast<uint32_t>(dds_.domain_id),
    participant_qos,
    cyclonedds_local_config(dds_.domain_id, dds_.interface));

  unitree::common::DdsPublisherQos publisher_qos;
  publisher_ = std::make_shared<unitree::common::DdsPublisher>(participant_, publisher_qos);

  unitree::common::DdsTopicQos topic_qos;
  channel_ = std::make_shared<unitree::common::DdsTopicChannel<RefMsg>>();
  channel_->SetTopic(participant_, dds_.topic, topic_qos);

  unitree::common::DdsWriterQos writer_qos;
  channel_->SetWriter(publisher_, writer_qos);

  spdlog::info(
    "WBC reference DDS publish: domain {} on {} topic '{}'",
    dds_.domain_id,
    dds_.interface,
    dds_.topic);
}

bool WbcReferencePublisher::publish(
  const std::vector<float>& arc,
  const std::vector<float>& joint_pos,
  const std::string& mode,
  uint64_t step,
  const std::string& robot_id,
  const std::string& clip_name,
  uint64_t episode)
{
  if (static_cast<int>(arc.size()) != DdsMotionReference::kArcDim) {
    throw std::invalid_argument(
      "WbcReferencePublisher::publish expects arc size 39, got " +
      std::to_string(arc.size()));
  }

  RefMsg msg;
  msg.version(1);
  msg.schema("wbc_reference_v1");
  msg.mode(mode);
  msg.robot_id(robot_id);
  msg.stamp_us(now_us());
  msg.step(step);

  std::vector<wbc_g1_deploy::msg::dds_::MetaKV_> meta;
  {
    wbc_g1_deploy::msg::dds_::MetaKV_ kv;
    kv.name(clip_name.empty() ? mode : clip_name);
    kv.values({static_cast<float>(episode)});
    meta.push_back(std::move(kv));
  }
  msg.meta(std::move(meta));
  msg.arc(arc);

  if (joint_pos.empty()) {
    msg.joint_pos(std::vector<float>(
      arc.begin() + DdsMotionReference::kJointOffset, arc.end()));
  } else {
    if (static_cast<int>(joint_pos.size()) != DdsMotionReference::kJointDim) {
      throw std::invalid_argument(
        "WbcReferencePublisher::publish expects joint_pos size 29, got " +
        std::to_string(joint_pos.size()));
    }
    msg.joint_pos(joint_pos);
  }

  if (!channel_->Write(msg, 0)) {
    ++sent_fail_;
    if (sent_fail_ == 1 || sent_fail_ % 50 == 0) {
      spdlog::warn("WBC reference DDS write failed (failures={})", sent_fail_);
    }
    return false;
  }

  ++sent_ok_;
  if (sent_ok_ == 1) {
    spdlog::info("WBC reference DDS first message sent");
  } else if (sent_ok_ % 250 == 0) {
    spdlog::debug("WBC reference DDS sent count={}", sent_ok_);
  }
  return true;
}

}  // namespace wbc_deploy
