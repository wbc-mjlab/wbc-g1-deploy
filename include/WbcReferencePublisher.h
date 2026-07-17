#pragma once

#include "DdsMotionReference.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace unitree {
namespace common {
class DdsParticipant;
class DdsPublisher;
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

/// Publish Arc references on the isolated WBC DDS bus (domain 101 by default).
/// Used by ``wbc_ref_pub`` and (later) ``wbc_reference_node``.
class WbcReferencePublisher
{
public:
  explicit WbcReferencePublisher(ReferenceDdsConfig dds = {});

  /// ``episode`` increments when the operator starts a new clip / Gen segment;
  /// the ctrl subscriber resets policy history on change.
  bool publish(
    const std::vector<float>& arc,
    const std::vector<float>& joint_pos = {},
    const std::string& mode = "stream",
    uint64_t step = 0,
    const std::string& robot_id = "g1",
    const std::string& clip_name = "",
    uint64_t episode = 0);

  const ReferenceDdsConfig& dds_config() const { return dds_; }
  uint64_t sent_ok() const { return sent_ok_; }
  uint64_t sent_fail() const { return sent_fail_; }

private:
  ReferenceDdsConfig dds_;
  std::shared_ptr<unitree::common::DdsParticipant> participant_;
  std::shared_ptr<unitree::common::DdsPublisher> publisher_;
  std::shared_ptr<unitree::common::DdsTopicChannel<wbc_g1_deploy::msg::dds_::WbcReference_>>
    channel_;
  uint64_t sent_ok_ = 0;
  uint64_t sent_fail_ = 0;
};

/// Canonical G1 standing Arc (39) matching wbc-gen ``G1_STATIC_REFERENCE_FLAT``.
std::vector<float> g1_standing_arc();

}  // namespace wbc_deploy
