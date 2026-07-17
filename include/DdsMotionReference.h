#pragma once

#include "IMotionReference.h"

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace unitree {
namespace common {
class DdsParticipant;
class DdsSubscriber;
template <typename MSG>
class DdsTopicChannel;
}  // namespace common
}  // namespace unitree

namespace wbc_g1_deploy {
namespace msg {
namespace dds_ {
class WbcReference_;
}
}  // namespace msg
}  // namespace wbc_g1_deploy

namespace wbc_deploy {

struct ReferenceDdsConfig {
  int domain_id = 101;
  std::string interface = "lo";
  std::string topic = "wbc/ref/command";
};

/// DDS-backed motion reference on a **separate** Cyclone domain (default 101),
/// mirroring ``estimators_deploy`` isolation from Unitree domain 0.
///
/// Wire type: ``wbc_g1_deploy::msg::dds_::WbcReference_``
/// Topic default: ``wbc/ref/command``
/// Payload: ``arc`` = 39-D Arc reference; optional ``joint_pos`` = 29 absolute q_ref
/// (if empty, joints are sliced from ``arc``).
class DdsMotionReference : public IMotionReference
{
public:
  static constexpr int kArcDim = 39;
  static constexpr int kJointDim = 29;
  static constexpr int kJointOffset = 10;  // height(1)+lin(3)+ang(3)+grav(3)

  explicit DdsMotionReference(
    std::vector<float> default_joint_pos,
    ReferenceDdsConfig dds = {},
    float step_dt = 0.02f);

  ~DdsMotionReference() override;

  void update(float time) override;
  void reset(const isaaclab::ArticulationData& data, float t = 0.0f) override;

  Eigen::VectorXf joint_pos() const override;
  float ref_base_height(float env_origin_z) const override;
  std::vector<float> ref_base_lin_vel_b() const override;
  std::vector<float> ref_base_ang_vel_b() const override;
  std::vector<float> ref_gravity_b() const override;
  std::vector<float> ref_joint_pos() const override;
  std::vector<float> ref_joint_vel() const override;

  float duration() const override;
  int frame() const override;

  bool has_sample() const { return has_sample_.load(); }
  const ReferenceDdsConfig& dds_config() const { return dds_; }
  std::string mode() const;
  std::string clip_name() const;
  uint64_t episode() const;
  bool consume_restart() override;

  /// Apply a packed Arc sample (unit tests / local inject).
  void applyArcCommand(
    const float* data,
    int n,
    const float* joint_pos = nullptr,
    int joint_n = 0,
    uint64_t time_frame = 0,
    const std::string& mode = "stream",
    uint64_t episode = 0,
    const std::string& clip_name = "");

private:
  void onMessage(const void* msg);
  void noteEpisodeLocked(uint64_t episode);
  std::array<float, kArcDim> copyArc() const;
  std::array<float, kJointDim> copyJoints() const;

  ReferenceDdsConfig dds_;
  float step_dt_;
  mutable std::mutex mtx_;
  std::array<float, kArcDim> arc_{};
  std::array<float, kJointDim> joints_{};
  bool joints_from_msg_ = false;
  uint64_t time_frame_ = 0;
  std::string mode_ = "stream";
  std::string clip_name_;
  uint64_t episode_ = 0;
  bool have_episode_ = false;
  bool restart_pending_ = false;
  std::atomic<bool> has_sample_{false};

  std::shared_ptr<unitree::common::DdsParticipant> participant_;
  std::shared_ptr<unitree::common::DdsSubscriber> subscriber_;
  std::shared_ptr<unitree::common::DdsTopicChannel<wbc_g1_deploy::msg::dds_::WbcReference_>>
    channel_;
};

std::string cyclonedds_local_config(int domain_id, const std::string& iface);
void apply_cyclonedds_env(int domain_id, const std::string& iface);

ReferenceDdsConfig load_reference_dds_config(const YAML::Node& root, const YAML::Node& fsm_cfg);

}  // namespace wbc_deploy
