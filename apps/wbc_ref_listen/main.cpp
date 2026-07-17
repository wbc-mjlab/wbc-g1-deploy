/// Listen for WBC Arc references on the isolated DDS bus (domain 101).

#include "DdsMotionReference.h"
#include "idl/WbcReference.hpp"
#include "param.h"

#include <unitree/common/dds/dds_entity.hpp>
#include <unitree/common/dds/dds_qos.hpp>
#include <unitree/common/dds/dds_topic_channel.hpp>

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <mutex>
#include <thread>

namespace {

using RefMsg = wbc_g1_deploy::msg::dds_::WbcReference_;

volatile std::sig_atomic_t g_running = 1;

void on_signal(int)
{
  g_running = 0;
}

}  // namespace

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  param::bin_path = param::get_bin_path();
  param::load_config_file();

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const auto dds = wbc_deploy::load_reference_dds_config(param::config, YAML::Node{});
  wbc_deploy::apply_cyclonedds_env(dds.domain_id, dds.interface);

  unitree::common::DdsParticipantQos participant_qos;
  const auto participant = std::make_shared<unitree::common::DdsParticipant>(
    static_cast<uint32_t>(dds.domain_id),
    participant_qos,
    wbc_deploy::cyclonedds_local_config(dds.domain_id, dds.interface));

  unitree::common::DdsSubscriberQos subscriber_qos;
  const auto subscriber =
    std::make_shared<unitree::common::DdsSubscriber>(participant, subscriber_qos);

  unitree::common::DdsTopicQos topic_qos;
  auto channel = std::make_shared<unitree::common::DdsTopicChannel<RefMsg>>();
  channel->SetTopic(participant, dds.topic, topic_qos);

  std::mutex sample_mutex;
  RefMsg latest;
  std::atomic<bool> has_sample{false};

  unitree::common::DdsReaderQos reader_qos;
  unitree::common::DdsReaderCallback reader_cb([&](const void* data) {
    const auto* msg = static_cast<const RefMsg*>(data);
    std::lock_guard<std::mutex> lock(sample_mutex);
    latest = *msg;
    has_sample.store(true);
  });
  channel->SetReader(subscriber, reader_qos, reader_cb, 16);

  spdlog::info(
    "Listening WBC refs: domain {} on {} topic '{}'",
    dds.domain_id,
    dds.interface,
    dds.topic);

  while (g_running) {
    if (!has_sample.exchange(false)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    RefMsg msg;
    {
      std::lock_guard<std::mutex> lock(sample_mutex);
      msg = latest;
    }
    const float h = msg.arc().empty() ? 0.0f : msg.arc()[0];
    std::string clip;
    if (!msg.meta().empty()) {
      clip = msg.meta()[0].name();
    }
    spdlog::info(
      "step={} mode={} clip={} arc_len={} h={:.3f}",
      msg.step(),
      msg.mode(),
      clip.empty() ? "-" : clip,
      msg.arc().size(),
      h);
  }
  return 0;
}
