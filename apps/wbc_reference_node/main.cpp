/// Unified reference node: NPZ clips + Gen ONNX on domain 101.
///
/// Clip UX matches ``State_WbcTracking`` (RT+D-pad / A / getup / liedown).
/// Switch to Gen with ``mode_gen`` (default ``RT + Y``); back with ``mode_clips``
/// (default ``RT + X``). In Gen, D-pad up/down steps height; RB+Y resets to idle
/// (0.80 m). Getup/liedown only from clips mode.
///
/// Examples::
///
///   ./wbc_reference_node -n eth0
///   ./wbc_reference_node --mode gen -n eth0
///   ./wbc_reference_node --dry-run --vx 0.5   # stdin: G/c n/p/Enter u/d/+/-/H

#include "GenReferenceEngine.h"
#include "MotionClipLibrary.h"
#include "Types.h"
#include "WbcReferencePublisher.h"
#include "gen_obs_builder.h"
#include "joystick_expr.h"
#include "pack_arc_reference.h"
#include "param.h"
#include "unitree_articulation.h"

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>

namespace po = boost::program_options;

namespace {

volatile std::sig_atomic_t g_running = 1;

void on_signal(int)
{
  g_running = 0;
}

enum class ActiveMode { Clips, Gen };

isaaclab::ArticulationData empty_articulation_data()
{
  return {};
}

float clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}

/// Stick ∈ [-1, 1] → asymmetric cruise velocity (0→0, +1→hi, −1→lo).
float stick_to_cruise(float stick, float lo, float hi)
{
  if (stick >= 0.0f) {
    return stick * hi;
  }
  return stick * (-lo);  // lo typically negative
}

/// RT ∈ [0, 1] → velocity boost ∈ [boost_min, boost_max].
float rt_vel_boost(float rt01, float boost_min, float boost_max)
{
  const float t = clampf(rt01, 0.0f, 1.0f);
  return boost_min + t * (boost_max - boost_min);
}

std::string yaml_str(const YAML::Node& node, const char* key, const std::string& def)
{
  if (node && node[key]) {
    return node[key].as<std::string>();
  }
  return def;
}

wbc_deploy::GenProprioSample proprio_from_articulation(
  const isaaclab::ArticulationData& data)
{
  wbc_deploy::GenProprioSample s;
  s.set(
    "base_ang_vel",
    {
      data.root_ang_vel_b.x(),
      data.root_ang_vel_b.y(),
      data.root_ang_vel_b.z(),
    });
  s.set(
    "projected_gravity",
    {
      data.projected_gravity_b.x(),
      data.projected_gravity_b.y(),
      data.projected_gravity_b.z(),
    });
  const int n = static_cast<int>(data.joint_pos.size());
  std::vector<float> joint_pos_rel(static_cast<size_t>(n));
  std::vector<float> joint_vel_rel(static_cast<size_t>(n));
  std::vector<float> joint_torque(static_cast<size_t>(n), 0.0f);
  for (int i = 0; i < n; ++i) {
    const float q0 =
      (i < data.default_joint_pos.size()) ? data.default_joint_pos[i] : 0.0f;
    joint_pos_rel[static_cast<size_t>(i)] = data.joint_pos[i] - q0;
    joint_vel_rel[static_cast<size_t>(i)] =
      (i < data.joint_vel.size()) ? data.joint_vel[i] : 0.0f;
    if (i < data.joint_torque.size()) {
      joint_torque[static_cast<size_t>(i)] = data.joint_torque[i];
    }
  }
  s.set("joint_pos_rel", std::move(joint_pos_rel));
  s.set("joint_vel_rel", std::move(joint_vel_rel));
  s.set("joint_torque", std::move(joint_torque));
  return s;
}

struct ClipUiState {
  /// Standing: idle / browse / Gen allowed.
  /// Down: only getup may be published or played (floor start or after liedown).
  enum class BodyState { Standing, Down };

  BodyState body = BodyState::Standing;
  bool awaiting_select = true;
  bool playing = false;
  float play_t = 0.0f;
  bool playback_finished = false;
  bool was_playback_finished = false;

  bool is_up() const { return body == BodyState::Standing; }
  bool is_down() const { return body == BodyState::Down; }
};

}  // namespace

int main(int argc, char** argv)
{
  param::bin_path = param::get_bin_path();
  param::load_config_file();
  const auto& root = param::config;
  const YAML::Node ref_cfg = root["reference_node"] ? root["reference_node"] : YAML::Node{};
  const YAML::Node track_cfg =
    (root["FSM"] && root["FSM"]["Wbc_Tracking"]) ? root["FSM"]["Wbc_Tracking"] : YAML::Node{};

  std::string mode_str = yaml_str(ref_cfg, "mode", "clips");
  double hz = ref_cfg["hz"] ? ref_cfg["hz"].as<double>() : 50.0;
  bool loop_browsable = ref_cfg["loop"] ? ref_cfg["loop"].as<bool>() : false;
  std::string clip_name = yaml_str(ref_cfg, "clip", "");
  bool autoplay = ref_cfg["autoplay"] ? ref_cfg["autoplay"].as<bool>() : false;

  po::options_description desc(
    "wbc_reference_node — unified clips + gen (joystick mode switch)");
  desc.add_options()
    ("help,h", "produce help message")
    ("mode", po::value<std::string>(&mode_str)->default_value(mode_str),
     "initial mode: clips | gen")
    ("clip", po::value<std::string>(&clip_name), "start browsable clip name")
    ("loop",
     po::value<bool>(&loop_browsable)->default_value(loop_browsable)->implicit_value(true),
     "loop browsable clips (skills never loop)")
    ("hz", po::value<double>(&hz)->default_value(hz), "publish rate")
    ("no-autoplay", po::bool_switch(), "start in clip-select (no auto play)")
    ("vx", po::value<float>()->default_value(0.0f), "default / dry-run lin_vel_x")
    ("vy", po::value<float>()->default_value(0.0f), "default / dry-run lin_vel_y")
    ("wz", po::value<float>()->default_value(0.0f), "default / dry-run ang_vel_z")
    ("dry-run", po::bool_switch(), "no Unitree lowstate (stdin + standing proprio)")
    ("verbose", "debug logs (periodic step status, joy edges)")
    ("network,n", po::value<std::string>()->default_value(""), "Unitree DDS network iface");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    std::cout
      << "\nJoystick (same as WBC tracking clips):\n"
      << "  LT+up / LT+down  Standing (idle) / Down (getup frame 0)\n"
      << "  RT+A on ctrl     enable policy (stand→idle ref, floor→getup frame 0)\n"
      << "  While Down:      RT+up = getup only (A/browse/Gen blocked)\n"
      << "  While Standing:  RT+L/R browse, A play (!RT), RT+Y Gen, RT+down=liedown\n"
      << "  RT+X             return to idle stand hold from Gen\n"
      << "Gen mode:\n"
      << "  L-stick Y/X   vx forward/back, vy strafe\n"
      << "  R-stick X     wz yaw; hold RT to boost lin+ang vel\n"
      << "  Gen height:      D-pad up/down without RT (only while Gen active)\n"
      << "  RB+Y           reset height to idle default\n"
      << "Getup/liedown always force clips mode (from clips; use RT+X then up/down).\n"
      << "Logging: quiet by default (mode/clip changes only); --verbose for step status.\n";
    return 0;
  }
  if (vm["no-autoplay"].as<bool>()) {
    autoplay = false;
  }
  spdlog::set_level(
    vm.count("verbose") ? spdlog::level::debug : spdlog::level::info);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  ActiveMode mode = (mode_str == "gen") ? ActiveMode::Gen : ActiveMode::Clips;
  if (mode_str != "clips" && mode_str != "gen") {
    spdlog::error("Unknown --mode '{}' (use clips|gen)", mode_str);
    return 1;
  }

  const float step_dt = static_cast<float>(1.0 / std::max(hz, 1e-3));
  const bool dry_run = vm["dry-run"].as<bool>();
  const std::string network = vm["network"].as<std::string>();
  float default_vx = vm["vx"].as<float>();
  float default_vy = vm["vy"].as<float>();
  float default_wz = vm["wz"].as<float>();

  // --- clip library ---
  std::filesystem::path clips_dir =
    param::config_string(root, "clips_dir", "config/clips");
  clips_dir = param::resolve_path_under_proj(clips_dir);
  std::filesystem::path manifest =
    param::config_string(root, "clips_manifest", "manifest.yaml");
  if (!manifest.is_absolute()) {
    manifest = clips_dir / manifest;
  }
  std::unordered_map<std::string, std::string> pose_clips;
  // Prefer reference_node.pose_clips; fall back to legacy FSM.Wbc_Tracking.
  const YAML::Node pose_src =
    (ref_cfg["pose_clips"] ? ref_cfg["pose_clips"] : track_cfg["pose_clips"]);
  if (pose_src) {
    for (const auto& entry : pose_src) {
      pose_clips[entry.first.as<std::string>()] = entry.second.as<std::string>();
    }
  }
  MotionClipLibrary library(clips_dir, manifest, step_dt, "torso_link", pose_clips);
  if (!clip_name.empty() && !library.selectBrowsableByName(clip_name, true)) {
    spdlog::error("Unknown clip '{}'", clip_name);
    return 1;
  }

  // --- gen engine (optional if params missing and never entering gen) ---
  std::filesystem::path gen_dir = param::config_string(
    root,
    "gen_dir",
    param::config_string(ref_cfg, "gen_dir", "config/policy/gen"));
  gen_dir = param::resolve_path_under_proj(gen_dir);
  std::unique_ptr<wbc_deploy::GenReferenceEngine> gen;
  const int infer_log_every =
    ref_cfg["infer_log_every"] ? ref_cfg["infer_log_every"].as<int>() : 0;
  try {
    gen = std::make_unique<wbc_deploy::GenReferenceEngine>(
      gen_dir / "params", infer_log_every);
    gen->reset();
  } catch (const std::exception& exc) {
    if (mode == ActiveMode::Gen) {
      spdlog::error("Gen required but failed to load: {}", exc.what());
      return 1;
    }
    spdlog::warn("Gen unavailable ({}). Clips-only until params installed.", exc.what());
  }

  // Prep pose for policy start (same buttons as Passive → FixStand / FloorReady).
  // Stand: hold first idle frame. Floor: hold getup frame 0 (lying).
  // Policy enable on ctrl is always RT+A from either prep state.
  const std::string expr_stand_prep = yaml_str(
    ref_cfg,
    "stand_prep",
    "LT + up.on_pressed");
  const std::string expr_floor_prep = yaml_str(
    ref_cfg,
    "floor_prep",
    "LT + down.on_pressed");
  // Joystick exprs — match ctrl; mode switch configurable on reference_node.
  const std::string expr_next =
    yaml_str(ref_cfg, "clip_next", yaml_str(track_cfg, "clip_next", "RT + right.on_pressed"));
  const std::string expr_prev =
    yaml_str(ref_cfg, "clip_prev", yaml_str(track_cfg, "clip_prev", "RT + left.on_pressed"));
  const std::string expr_play =
    yaml_str(ref_cfg, "clip_play", yaml_str(track_cfg, "clip_play", "!RT + A.on_pressed"));
  // Down-only getup (after FloorReady / liedown). RT+A enables policy; RT+up plays getup.
  const std::string expr_getup =
    yaml_str(ref_cfg, "clip_getup", yaml_str(track_cfg, "clip_getup", "RT + up.on_pressed"));
  const std::string expr_liedown =
    yaml_str(ref_cfg, "clip_liedown", yaml_str(track_cfg, "clip_liedown", "RT + down.on_pressed"));
  const std::string expr_mode_gen =
    yaml_str(ref_cfg, "mode_gen", "RT + Y.on_pressed");
  const std::string expr_mode_clips =
    yaml_str(ref_cfg, "mode_clips", "RT + X.on_pressed");
  // Gen-only height: require !RT so RT+up / RT+down stay getup / liedown.
  const std::string expr_height_up =
    yaml_str(ref_cfg, "height_up", "!RT + up.on_pressed");
  const std::string expr_height_down =
    yaml_str(ref_cfg, "height_down", "!RT + down.on_pressed");
  const std::string expr_height_reset =
    yaml_str(ref_cfg, "height_reset", "RB + Y.on_pressed");
  const float height_step =
    ref_cfg["height_step"] ? ref_cfg["height_step"].as<float>() : 0.05f;

  auto clip_next_fn = compileJoystickExpr(expr_next);
  auto clip_prev_fn = compileJoystickExpr(expr_prev);
  auto clip_play_fn = compileJoystickExpr(expr_play);
  auto clip_getup_fn = compileJoystickExpr(expr_getup);
  auto clip_liedown_fn = compileJoystickExpr(expr_liedown);
  auto mode_gen_fn = compileJoystickExpr(expr_mode_gen);
  auto mode_clips_fn = compileJoystickExpr(expr_mode_clips);
  auto height_up_fn = compileJoystickExpr(expr_height_up);
  auto height_down_fn = compileJoystickExpr(expr_height_down);
  auto height_reset_fn = compileJoystickExpr(expr_height_reset);
  auto stand_prep_fn = compileJoystickExpr(expr_stand_prep);
  auto floor_prep_fn = compileJoystickExpr(expr_floor_prep);

  // --- Unitree lowstate FIRST (joystick + Gen proprio), then DDS pub ---
  // Creating the domain-101 publisher must not run before ChannelFactory::Init,
  // and must not setenv CYCLONEDDS_URI (see WbcReferencePublisher).
  std::shared_ptr<LowState_t> lowstate;
  std::unique_ptr<unitree::BaseArticulation<std::shared_ptr<LowState_t>>> robot;
  if (!dry_run) {
    if (network.empty()) {
      spdlog::warn(
        "No --network/-n set; Unitree lowstate (and joystick) may not connect. "
        "Example: ./wbc_reference_node -n eth0");
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
    lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for Unitree lowstate (joystick) on iface '{}'...", network);
    lowstate->wait_for_connection();
    robot = std::make_unique<unitree::BaseArticulation<std::shared_ptr<LowState_t>>>(lowstate);
    if (gen) {
      robot->data.joint_ids_map.clear();
      for (int id : gen->params().joint_ids_map) {
        robot->data.joint_ids_map.push_back(static_cast<float>(id));
      }
      robot->data.default_joint_pos = Eigen::Map<const Eigen::VectorXf>(
        gen->params().default_joint_pos.data(),
        static_cast<int>(gen->params().default_joint_pos.size()));
    } else {
      // Minimal 29-DoF identity map for joystick-only clips.
      robot->data.joint_ids_map.resize(29);
      for (int i = 0; i < 29; ++i) {
        robot->data.joint_ids_map[static_cast<size_t>(i)] = static_cast<float>(i);
      }
      robot->data.default_joint_pos = Eigen::VectorXf::Zero(29);
    }
    robot->data.joint_pos = robot->data.default_joint_pos;
    robot->data.joint_vel = Eigen::VectorXf::Zero(robot->data.joint_pos.size());
    robot->data.joint_torque = Eigen::VectorXf::Zero(robot->data.joint_pos.size());
    spdlog::info("Connected to robot lowstate (joystick ready)");
  } else {
    spdlog::info(
      "Dry-run: stdin (n/p/Enter browse/play; u/d getup|height; +/- height; "
      "H idle height; G=gen C=clips q)");
  }

  const auto dds = wbc_deploy::load_reference_dds_config(root, YAML::Node{});
  wbc_deploy::WbcReferencePublisher pub(dds);

  ClipUiState ui;
  // initial_up selects stand vs floor hold at boot (overridable by LT+up/down).
  const bool initial_up =
    ref_cfg["initial_up"] ? ref_cfg["initial_up"].as<bool>() : true;
  ui.body = initial_up ? ClipUiState::BodyState::Standing : ClipUiState::BodyState::Down;
  uint64_t episode = 0;
  // Clip/Gen hold applied after enter_* helpers are defined (see below).

  float play_vx_lo = -1.5f;
  float play_vx_hi = 4.0f;
  float play_vy_lo = -1.0f;
  float play_vy_hi = 1.0f;
  float play_wz_lo = -3.0f;
  float play_wz_hi = 3.0f;
  float height_lo = 0.55f;
  float height_hi = 0.95f;
  // Play idle ``DEFAULT_STAND_HEIGHT``; clamped to gen play_vel_ranges.height.
  float default_height = 0.80f;
  if (gen) {
    if (gen->params().play_vel_ranges.count("lin_vel_x")) {
      const auto& r = gen->params().play_vel_ranges.at("lin_vel_x");
      play_vx_lo = r.first;
      play_vx_hi = r.second;
    }
    if (gen->params().play_vel_ranges.count("lin_vel_y")) {
      const auto& r = gen->params().play_vel_ranges.at("lin_vel_y");
      play_vy_lo = r.first;
      play_vy_hi = r.second;
    }
    if (gen->params().play_vel_ranges.count("ang_vel_z")) {
      const auto& r = gen->params().play_vel_ranges.at("ang_vel_z");
      play_wz_lo = r.first;
      play_wz_hi = r.second;
    }
    if (gen->params().play_vel_ranges.count("height")) {
      const auto& hr = gen->params().play_vel_ranges.at("height");
      height_lo = hr.first;
      height_hi = hr.second;
      default_height = clampf(0.80f, height_lo, height_hi);
    }
  }

  const float vel_boost_min =
    ref_cfg["vel_boost_min"] ? ref_cfg["vel_boost_min"].as<float>() : 1.0f;
  const float vel_boost_max =
    ref_cfg["vel_boost_max"] ? ref_cfg["vel_boost_max"].as<float>() : 2.5f;
  auto cruise_pair = [&](const char* key, float lo_def, float hi_def) {
    if (ref_cfg[key] && ref_cfg[key].IsSequence() && ref_cfg[key].size() >= 2) {
      return std::make_pair(ref_cfg[key][0].as<float>(), ref_cfg[key][1].as<float>());
    }
    return std::make_pair(lo_def, hi_def);
  };
  const auto [cruise_vx_lo, cruise_vx_hi] =
    cruise_pair("cruise_lin_vel_x", -0.7f, 1.0f);
  const auto [cruise_vy_lo, cruise_vy_hi] =
    cruise_pair("cruise_lin_vel_y", -0.4f, 0.4f);
  const auto [cruise_wz_lo, cruise_wz_hi] =
    cruise_pair("cruise_ang_vel_z", -1.2f, 1.2f);

  float gen_height_cmd = default_height;

  spdlog::info(
    "Bindings: next='{}' prev='{}' play='{}' getup='{}' liedown='{}' "
    "stand_prep='{}' floor_prep='{}' "
    "mode_gen='{}' mode_clips='{}' height_up='{}' height_down='{}' "
    "height_reset='{}' (idle h={:.2f} step={:.2f} range=[{:.2f},{:.2f}])",
    expr_next,
    expr_prev,
    expr_play,
    expr_getup,
    expr_liedown,
    expr_stand_prep,
    expr_floor_prep,
    expr_mode_gen,
    expr_mode_clips,
    expr_height_up,
    expr_height_down,
    expr_height_reset,
    default_height,
    height_step,
    height_lo,
    height_hi);
  spdlog::info(
    "Gen vel: cruise vx=[{:.2f},{:.2f}] vy=[{:.2f},{:.2f}] wz=[{:.2f},{:.2f}] "
    "× RT boost [{:.2f},{:.2f}] → clamp play vx=[{:.2f},{:.2f}]",
    cruise_vx_lo,
    cruise_vx_hi,
    cruise_vy_lo,
    cruise_vy_hi,
    cruise_wz_lo,
    cruise_wz_hi,
    vel_boost_min,
    vel_boost_max,
    play_vx_lo,
    play_vx_hi);

  std::atomic<int> stdin_cmd{0};  // mirror joystick for dry-run
  std::thread stdin_thread;
  if (dry_run) {
    stdin_thread = std::thread([&] {
      std::string line;
      while (g_running) {
        if (!std::getline(std::cin, line)) {
          break;
        }
        if (line.empty() || line == "\r") {
          stdin_cmd.store(3);  // play
        } else if (line == "n" || line == "N") {
          stdin_cmd.store(1);
        } else if (line == "p" || line == "P") {
          stdin_cmd.store(2);
        } else if (line == "u" || line == "U") {
          stdin_cmd.store(4);  // getup (clips) / height up (gen)
        } else if (line == "d" || line == "D") {
          stdin_cmd.store(5);  // liedown (clips) / height down (gen)
        } else if (line == "+" || line == "=") {
          stdin_cmd.store(9);  // height up
        } else if (line == "-" || line == "_") {
          stdin_cmd.store(10);  // height down
        } else if (line == "H") {
          stdin_cmd.store(11);  // height reset to idle
        } else if (line == "G") {
          stdin_cmd.store(7);  // gen
        } else if (line == "C" || line == "c") {
          stdin_cmd.store(8);  // clips
        } else if (line == "q" || line == "Q") {
          g_running = 0;
          break;
        }
      }
    });
  }

  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(step_dt));
  auto next_wake = std::chrono::steady_clock::now();
  uint64_t step = 0;

  /// Stand hold / return to trajectory mode: always publish default idle frame 0
  /// first, then allow browse/play/Gen (only once Standing).
  auto enter_stand_hold = [&](const char* hit = nullptr, bool bump_episode = true) {
    mode = ActiveMode::Clips;
    if (!library.selectDefaultBrowsable()) {
      spdlog::error("Failed to load default stand idle clip");
      return;
    }
    ui.body = ClipUiState::BodyState::Standing;
    ui.awaiting_select = true;
    ui.playing = false;
    ui.play_t = 0.0f;
    ui.playback_finished = false;
    ui.was_playback_finished = false;
    if (bump_episode) {
      ++episode;
    }
    if (hit) {
      spdlog::info(
        "Hit: {} — Standing: {} frame 0 (browse/play or RT+Y Gen)",
        hit,
        library.currentName());
    } else {
      spdlog::info(
        "Standing: {} frame 0 (ctrl RT+A starts policy; RT+L/R browse, A play)",
        library.currentName());
    }
  };

  /// Down (FloorReady / after liedown): hold getup frame 0. Only getup may be
  /// published or played until Standing again — never idle / Gen / browse.
  auto enter_floor_hold = [&](const char* hit = nullptr, bool bump_episode = true) {
    mode = ActiveMode::Clips;
    if (!library.selectPoseClip("getup")) {
      spdlog::error("Failed to load getup pose for Down state");
      return;
    }
    ui.body = ClipUiState::BodyState::Down;
    ui.awaiting_select = false;
    ui.playing = false;
    ui.play_t = 0.0f;
    ui.playback_finished = false;
    ui.was_playback_finished = false;
    if (bump_episode) {
      ++episode;
    }
    if (hit) {
      spdlog::info(
        "Hit: {} — Down: {} frame 0 (RT+A = enable policy; RT+up = getup)",
        hit,
        library.currentName());
    } else {
      spdlog::info(
        "Down: {} frame 0 (RT+A = enable policy; RT+up = getup)",
        library.currentName());
    }
  };

  auto start_clip_playback = [&]() {
    mode = ActiveMode::Clips;
    // Safety: while Down, never start anything except getup.
    if (ui.is_down() &&
        library.currentKind() != MotionClipLibrary::ClipKind::PoseGetup) {
      if (!library.selectPoseClip("getup")) {
        spdlog::error("Down state refused non-getup playback");
        return;
      }
    }
    ui.awaiting_select = false;
    ui.playing = true;
    ui.play_t = 0.0f;
    ui.playback_finished = false;
    ui.was_playback_finished = false;
    ++episode;
    library.resetPlayback(empty_articulation_data(), 0.0f);
    spdlog::info("Playing clip: {}", library.currentName());
  };

  auto enter_gen = [&](const char* hit) {
    if (!gen) {
      spdlog::warn("Gen not loaded — staying in clips");
      return;
    }
    if (ui.is_down()) {
      spdlog::warn("Down — getup before Gen");
      return;
    }
    mode = ActiveMode::Gen;
    ui.playing = false;
    ui.awaiting_select = false;
    ++episode;
    gen->reset();
    gen_height_cmd = default_height;
    gen->seed_height(gen_height_cmd);
    if (robot) {
      robot->update();
      gen->push_proprio(proprio_from_articulation(robot->data));
      // Fill history quickly.
      auto sample = proprio_from_articulation(robot->data);
      for (int i = 0; i < 20; ++i) {
        gen->push_proprio(sample);
      }
    } else {
      auto sample = wbc_deploy::standing_proprio_sample(gen->params());
      for (int i = 0; i < 20; ++i) {
        gen->push_proprio(sample);
      }
    }
    spdlog::info(
      "Hit: {} — switched to Generator "
      "(L-stick: Y→vx forward/back, X→vy strafe; R-stick X→wz yaw; "
      "RT boosts lin+ang vel; D-pad height; RB+Y=idle {:.2f} m; RT+X → clips)",
      hit,
      default_height);
  };

  // Initial reference pose for policy bring-up (no episode bump yet).
  if (autoplay && mode == ActiveMode::Clips) {
    ui.awaiting_select = false;
    ui.playing = true;
    ui.play_t = 0.0f;
    ++episode;
    library.resetPlayback(empty_articulation_data(), 0.0f);
    spdlog::info("Autoplay: {}", library.currentName());
  } else if (mode == ActiveMode::Gen) {
    ui.awaiting_select = false;
    ui.playing = false;
    ++episode;
    spdlog::info("Starting in Gen mode");
  } else if (!initial_up) {
    enter_floor_hold(nullptr, false);
  } else {
    enter_stand_hold(nullptr, false);
  }

  while (g_running) {
    // Joystick lives in lowstate->wireless_remote; must call update() every tick
    // (same as FSMState::pre_run). robot->update() alone does not refresh joy.
    if (lowstate) {
      lowstate->update();
    }
    if (robot) {
      robot->update();
    }

    // --- input: joystick and/or stdin ---
    bool do_next = false;
    bool do_prev = false;
    bool do_play = false;
    bool do_getup = false;
    bool do_liedown = false;
    bool do_mode_gen = false;
    bool do_mode_clips = false;
    bool do_height_up = false;
    bool do_height_down = false;
    bool do_height_reset = false;
    bool do_stand_prep = false;
    bool do_floor_prep = false;
    float cmd_vx = default_vx;
    float cmd_vy = default_vy;
    float cmd_wz = default_wz;

    if (lowstate) {
      auto& joy = lowstate->joystick;
      do_next = clip_next_fn && clip_next_fn(joy);
      do_prev = clip_prev_fn && clip_prev_fn(joy);
      do_play = clip_play_fn && clip_play_fn(joy);
      do_getup = clip_getup_fn && clip_getup_fn(joy);
      do_liedown = clip_liedown_fn && clip_liedown_fn(joy);
      do_mode_gen = mode_gen_fn && mode_gen_fn(joy);
      do_mode_clips = mode_clips_fn && mode_clips_fn(joy);
      // Height teleop only while Generator is active (never in clips / Down).
      if (mode == ActiveMode::Gen) {
        do_height_up = height_up_fn && height_up_fn(joy);
        do_height_down = height_down_fn && height_down_fn(joy);
        do_height_reset = height_reset_fn && height_reset_fn(joy);
      }
      do_stand_prep = stand_prep_fn && stand_prep_fn(joy);
      do_floor_prep = floor_prep_fn && floor_prep_fn(joy);
      // Cruise stick × RT boost (1 → vel_boost_max), then clamp to play ranges.
      const float boost =
        rt_vel_boost(joy.RT(), vel_boost_min, vel_boost_max);
      cmd_vx = clampf(
        stick_to_cruise(joy.ly(), cruise_vx_lo, cruise_vx_hi) * boost,
        play_vx_lo,
        play_vx_hi);
      cmd_vy = clampf(
        stick_to_cruise(-joy.lx(), cruise_vy_lo, cruise_vy_hi) * boost,
        play_vy_lo,
        play_vy_hi);
      cmd_wz = clampf(
        stick_to_cruise(-joy.rx(), cruise_wz_lo, cruise_wz_hi) * boost,
        play_wz_lo,
        play_wz_hi);

      if (do_next || do_prev || do_play || do_getup || do_liedown || do_mode_gen ||
          do_mode_clips || do_height_up || do_height_down || do_height_reset ||
          do_stand_prep || do_floor_prep) {
        spdlog::debug(
          "joy edge: next={} prev={} play={} getup={} liedown={} gen={} clips={} "
          "stand={} floor={} h_up={} h_down={} h_reset={} RT={} A={} Y={} X={} "
          "up={} down={}",
          do_next,
          do_prev,
          do_play,
          do_getup,
          do_liedown,
          do_mode_gen,
          do_mode_clips,
          do_stand_prep,
          do_floor_prep,
          do_height_up,
          do_height_down,
          do_height_reset,
          joy.RT.pressed,
          joy.A.pressed,
          joy.Y.pressed,
          joy.X.pressed,
          joy.up.pressed,
          joy.down.pressed);
      }
    }

    const int sc = stdin_cmd.exchange(0);
    if (sc == 1) {
      do_next = true;
    } else if (sc == 2) {
      do_prev = true;
    } else if (sc == 3) {
      do_play = true;
    } else if (sc == 4) {
      // Clips Down: getup; Gen: height up.
      if (mode == ActiveMode::Gen) {
        do_height_up = true;
      } else {
        do_getup = true;
      }
    } else if (sc == 5) {
      if (mode == ActiveMode::Gen) {
        do_height_down = true;
      } else {
        do_liedown = true;
      }
    } else if (sc == 7) {
      do_mode_gen = true;
    } else if (sc == 8) {
      do_mode_clips = true;
    } else if (sc == 9) {
      if (mode == ActiveMode::Gen) {
        do_height_up = true;
      }
    } else if (sc == 10) {
      if (mode == ActiveMode::Gen) {
        do_height_down = true;
      }
    } else if (sc == 11) {
      if (mode == ActiveMode::Gen) {
        do_height_reset = true;
      }
    }

    // Prep holds mirror Passive → FixStand / FloorReady so Arc matches robot pose
    // before the single policy enable (ctrl RT+A).
    // BodyState::Down: only getup — never idle / Gen / browse.
    //
    // Heal: frozen getup pose always means Down (covers missed LT+down while
    // ctrl already entered FloorReady, or boot --mode gen left body=Standing).
    if (mode == ActiveMode::Clips && !ui.playing && library.loader() &&
        library.currentKind() == MotionClipLibrary::ClipKind::PoseGetup) {
      ui.body = ClipUiState::BodyState::Down;
      ui.awaiting_select = false;
    }

    if (do_stand_prep) {
      if (ui.is_down()) {
        spdlog::info("Down — getup first (RT+up); stand prep blocked");
      } else {
        enter_stand_hold("LT+up");
      }
    } else if (do_floor_prep) {
      enter_floor_hold("LT+down");
    } else if (do_getup && mode == ActiveMode::Gen) {
      // Started in Gen or still Gen while ctrl is on FloorReady: RT+up = getup.
      spdlog::info("Hit: RT+up — leave Gen, play getup");
      if (library.selectPoseClip("getup")) {
        ui.body = ClipUiState::BodyState::Down;
        start_clip_playback();
      }
    } else if (mode == ActiveMode::Clips) {
      auto loader = library.loader();
      const float duration = loader ? loader->duration() : 0.0f;
      ui.was_playback_finished = ui.playback_finished;
      ui.playback_finished =
        ui.playing && (ui.play_t >= duration - 0.5f * step_dt);

      if (ui.playback_finished && !ui.was_playback_finished) {
        const auto kind = library.currentKind();
        if (kind == MotionClipLibrary::ClipKind::PoseGetup) {
          // Getup done → Standing: idle hold, then Gen/browse/play OK.
          enter_stand_hold();
        } else if (kind == MotionClipLibrary::ClipKind::PoseLiedown) {
          // Liedown done → Down: getup frame 0 only until getup plays.
          enter_floor_hold("liedown done");
        } else if (kind == MotionClipLibrary::ClipKind::Browsable) {
          if (loop_browsable) {
            ui.play_t = 0.0f;
            library.resetPlayback(empty_articulation_data(), 0.0f);
          } else {
            enter_stand_hold();
          }
        }
      }

      if (ui.is_down()) {
        // Down: RT+A enables policy on ctrl only; RT+up plays getup.
        if (!ui.playing && do_getup) {
          if (library.selectPoseClip("getup")) {
            start_clip_playback();
          }
        } else if (do_mode_gen || do_next || do_prev || do_play || do_liedown) {
          spdlog::info(
            "Down — hold getup frame 0; press RT+up to getup "
            "(RT+A = policy enable only)");
        }
      } else {
        if (do_mode_gen) {
          enter_gen(sc == 7 ? "stdin G" : "RT+Y");
        } else if (
          do_liedown && (ui.awaiting_select || ui.playback_finished) &&
          library.selectPoseClip("liedown")) {
          start_clip_playback();
        } else if (ui.awaiting_select) {
          if (do_next) {
            library.browseNextSelected();
          } else if (do_prev) {
            library.browsePrevSelected();
          } else if (do_play && library.activateSelectedBrowsable()) {
            start_clip_playback();
          }
        }
      }
      // Height is Gen-only; never consume D-pad here.
    } else {  // Gen — height teleop active only in this branch
      if (do_mode_clips) {
        enter_stand_hold(sc == 8 ? "stdin C" : "RT+X");
      } else if (do_height_reset) {
        const float before = gen_height_cmd;
        gen_height_cmd = default_height;
        if (gen) {
          gen->seed_height(gen_height_cmd);
        }
        if (before != gen_height_cmd) {
          spdlog::info(
            "Gen height RESET {:.3f} → {:.3f} m (idle, seeded)",
            before,
            gen_height_cmd);
        }
      } else if (do_height_up) {
        const float before = gen_height_cmd;
        gen_height_cmd = clampf(gen_height_cmd + height_step, height_lo, height_hi);
        if (gen) {
          gen->set_height_cmd(gen_height_cmd);
        }
        if (before != gen_height_cmd) {
          spdlog::info("Gen height ↑ {:.3f} → {:.3f} m", before, gen_height_cmd);
        }
      } else if (do_height_down) {
        const float before = gen_height_cmd;
        gen_height_cmd = clampf(gen_height_cmd - height_step, height_lo, height_hi);
        if (gen) {
          gen->set_height_cmd(gen_height_cmd);
        }
        if (before != gen_height_cmd) {
          spdlog::info("Gen height ↓ {:.3f} → {:.3f} m", before, gen_height_cmd);
        }
      } else if (do_play) {
        enter_stand_hold(sc == 3 ? "stdin Enter" : "A");
      }
    }

    // --- publish ---
    std::vector<float> arc;
    std::string wire_mode;
    std::string clip_meta;

    if (mode == ActiveMode::Gen && gen) {
      wbc_deploy::GenProprioSample proprio =
        wbc_deploy::standing_proprio_sample(gen->params());
      if (robot) {
        proprio = proprio_from_articulation(robot->data);
      }
      gen->push_proprio(proprio);
      gen->set_height_cmd(gen_height_cmd);
      arc = gen->step(cmd_vx, cmd_vy, cmd_wz);
      wire_mode = "gen";
      clip_meta = "gen";
    } else {
      auto loader = library.loader();
      if (ui.playing) {
        loader->update(ui.play_t);
        ui.play_t += step_dt;
      } else {
        loader->update(ui.play_t);
      }
      arc = wbc_deploy::pack_arc_reference(*loader);
      wire_mode = "clips";
      clip_meta = library.currentName();
    }

    pub.publish(arc, {}, wire_mode, step, "g1", clip_meta, episode);
    ++step;
    float last_boost = 1.0f;
    if (lowstate) {
      last_boost = rt_vel_boost(lowstate->joystick.RT(), vel_boost_min, vel_boost_max);
    }
    if (step == 1 || step % 100 == 0) {
      const bool joy_to = lowstate && lowstate->isJoystickTimeout();
      spdlog::debug(
        "step={} active={} clip={} playing={} select={} episode={} joy_timeout={} "
        "vx={:.2f} boost={:.2f} height_cmd={:.3f} ref_h={:.3f}",
        step,
        mode == ActiveMode::Gen ? "gen" : "clips",
        clip_meta,
        ui.playing,
        ui.awaiting_select,
        episode,
        joy_to,
        cmd_vx,
        mode == ActiveMode::Gen ? last_boost : 1.0f,
        mode == ActiveMode::Gen ? gen_height_cmd : 0.0f,
        arc.empty() ? 0.0f : arc[0]);
    }

    next_wake += period;
    std::this_thread::sleep_until(next_wake);
  }

  if (stdin_thread.joinable()) {
    stdin_thread.detach();
  }
  spdlog::info("Stopped after {} publishes", pub.sent_ok());
  return 0;
}
