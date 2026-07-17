/// Publish standing Arc references on the isolated WBC DDS bus (C++).
///
/// Does **not** touch Unitree domain 0. Pair with ``wbc_g1_ctrl`` using
/// ``reference_source: dds``.
///
/// Example::
///
///   ./wbc_ref_pub
///   ./wbc_ref_pub --hz 50 --mode stream

#include "DdsMotionReference.h"
#include "WbcReferencePublisher.h"
#include "param.h"

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <csignal>
#include <thread>

namespace po = boost::program_options;

namespace {

volatile std::sig_atomic_t g_running = 1;

void on_signal(int)
{
  g_running = 0;
}

}  // namespace

int main(int argc, char** argv)
{
  // Load config.yaml via param (same as ctrl), then parse pub-specific flags.
  param::bin_path = param::get_bin_path();
  param::load_config_file();

  double hz = 50.0;
  std::string mode = "stream";
  if (param::config["reference_pub"]) {
    if (param::config["reference_pub"]["hz"]) {
      hz = param::config["reference_pub"]["hz"].as<double>();
    }
    if (param::config["reference_pub"]["mode"]) {
      mode = param::config["reference_pub"]["mode"].as<std::string>();
    }
  }

  po::options_description desc("wbc_ref_pub — publish Arc refs on domain 101");
  desc.add_options()
    ("help,h", "produce help message")
    ("hz", po::value<double>(&hz)->default_value(hz), "publish rate")
    ("mode", po::value<std::string>(&mode)->default_value(mode), "mode string on wire");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const auto dds = wbc_deploy::load_reference_dds_config(param::config, YAML::Node{});
  wbc_deploy::WbcReferencePublisher pub(dds);
  const auto arc = wbc_deploy::g1_standing_arc();

  spdlog::info(
    "Publishing standing Arc @ {:.1f} Hz (mode={}) — Ctrl+C to stop",
    hz,
    mode);

  uint64_t step = 0;
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / std::max(hz, 1e-3)));
  auto next = std::chrono::steady_clock::now();

  while (g_running) {
    pub.publish(arc, {}, mode, step);
    ++step;
    if (step == 1 || step % 50 == 0) {
      spdlog::info("sent step={} ok={} fail={}", step, pub.sent_ok(), pub.sent_fail());
    }
    next += period;
    std::this_thread::sleep_until(next);
  }

  spdlog::info("Stopped after {} sends", pub.sent_ok());
  return 0;
}
