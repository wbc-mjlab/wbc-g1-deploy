#include "State_WbcTracking.h"

#include "DdsMotionReference.h"
#include "IMotionReference.h"
#include "MotionClipLibrary.h"
#include "joystick_expr.h"
#include "pd_torque_clip.h"
#include "unitree_articulation.h"
#include "wbc_entry_mode.h"
#include "wbc_mdp_registrations.h"
#include "wbc_tracking_params.h"
#include "isaaclab/algorithms/algorithms.h"

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <unordered_map>

std::shared_ptr<IMotionReference> State_WbcTracking::motion = nullptr;

State_WbcTracking::~State_WbcTracking()
{
  exit();
}

State_WbcTracking::State_WbcTracking(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
  wbc_deploy::ensure_mdp_registered();

  const auto cfg = param::config["FSM"][state_string];
  const auto& root = param::config;
  auto policy_dir = param::resolve_policy_dir(
    param::config_string(cfg, "policy_dir",
      param::config_string(root, "policy_dir", "config/policy/wbc")));

  const YAML::Node deploy = wbc_deploy::load_policy_config(policy_dir);
  const float step_dt = deploy["step_dt"].as<float>();
  std::string anchor_body = "torso_link";
  if (deploy["wbc_tracking"] && deploy["wbc_tracking"]["anchor_body_name"]) {
    anchor_body = deploy["wbc_tracking"]["anchor_body_name"].as<std::string>();
  }

  reference_source_ = param::config_string(
    cfg, "reference_source",
    param::config_string(root, "reference_source", "dds"));
  if (reference_source_ != "clips" && reference_source_ != "dds") {
    throw std::runtime_error(
      "reference_source must be 'clips' or 'dds' (got '" + reference_source_ + "')");
  }

  if (reference_source_ == "dds") {
    std::vector<float> default_q;
    if (deploy["default_joint_pos"]) {
      default_q = deploy["default_joint_pos"].as<std::vector<float>>();
    }
    auto dds_cfg = wbc_deploy::load_reference_dds_config(root, cfg);
    motion = std::make_shared<wbc_deploy::DdsMotionReference>(
      std::move(default_q), dds_cfg, step_dt);
    spdlog::info(
      "WBC reference_source=dds (domain {} topic '{}') — "
      "clip/Gen UX owned by wbc_reference_node",
      dds_cfg.domain_id,
      dds_cfg.topic);
  } else {
    // Legacy single-process bring-up: local NPZ + in-ctrl joystick.
    std::filesystem::path clips_dir = param::config_string(
      cfg, "clips_dir", param::config_string(root, "clips_dir", "config/clips"));
    clips_dir = param::resolve_path_under_proj(clips_dir);

    std::filesystem::path manifest = param::config_string(
      cfg, "clips_manifest",
      param::config_string(root, "clips_manifest", "manifest.yaml"));
    if (!manifest.is_absolute()) {
      manifest = clips_dir / manifest;
    }

    std::unordered_map<std::string, std::string> pose_clips;
    if (cfg["pose_clips"]) {
      for (const auto& entry : cfg["pose_clips"]) {
        pose_clips[entry.first.as<std::string>()] = entry.second.as<std::string>();
      }
    }

    clip_library_ = std::make_unique<MotionClipLibrary>(
      clips_dir, manifest, step_dt, anchor_body, pose_clips);
    motion = clip_library_->loader();

    const std::string clip_next_expr =
      cfg["clip_next"].as<std::string>("RT + right.on_pressed");
    const std::string clip_prev_expr =
      cfg["clip_prev"].as<std::string>("RT + left.on_pressed");
    const std::string clip_play_expr =
      cfg["clip_play"].as<std::string>("A.on_pressed");
    const std::string clip_getup_expr =
      cfg["clip_getup"].as<std::string>("up.on_pressed");
    const std::string clip_liedown_expr =
      cfg["clip_liedown"].as<std::string>("down.on_pressed");
    clip_next_fn_ = compileJoystickExpr(clip_next_expr);
    clip_prev_fn_ = compileJoystickExpr(clip_prev_expr);
    clip_play_fn_ = compileJoystickExpr(clip_play_expr);
    clip_getup_fn_ = compileJoystickExpr(clip_getup_expr);
    clip_liedown_fn_ = compileJoystickExpr(clip_liedown_expr);
    spdlog::info("WBC reference_source=clips (legacy in-process library)");
  }

  auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
    FSMState::lowstate);

  env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
  env->alg = std::make_unique<isaaclab::OrtRunner>(
    wbc_deploy::resolve_onnx_path(policy_dir).string(), "wbc");

  pd_torque_clip_ = wbc_deploy::load_pd_torque_clip_config(
    cfg, env->robot->data.joint_ids_map.size());

  time_range_[0] = cfg["time_start"] ? cfg["time_start"].as<float>() : 0.0f;
  syncPlaybackEndToMotion();
}

void State_WbcTracking::syncPlaybackEndToMotion()
{
  if (!motion) {
    return;
  }
  time_range_[1] = motion->duration();
}

void State_WbcTracking::applyMotionLoader(bool reset_env)
{
  if (reference_source_ == "dds" || !clip_library_) {
    return;
  }
  motion = clip_library_->loader();
  syncPlaybackEndToMotion();
  beginClipPlayback();
  if (reset_env) {
    env->reset();
    clip_library_->resetPlayback(env->robot->data, time_range_[0]);
  }
  spdlog::info("Playing motion clip: {}", clip_library_->currentName());
}

void State_WbcTracking::beginClipPlayback()
{
  clip_playback_active_ = true;
  awaiting_clip_select_ = false;
  playback_finished_ = false;
  was_playback_finished_ = false;
}

void State_WbcTracking::pauseClipPlayback(float frozen_time)
{
  clip_playback_active_ = false;
  frozen_playback_time_ = frozen_time;
}

void State_WbcTracking::enterClipSelectMode(float frozen_time)
{
  awaiting_clip_select_ = true;
  pauseClipPlayback(frozen_time);
  playback_finished_ = false;
  was_playback_finished_ = false;
  spdlog::info(
    "Select clip with RT + D-pad left/right, play {} with A",
    clip_library_->selectedBrowsableName());
}

float State_WbcTracking::currentPlaybackTime() const
{
  if (clip_playback_active_) {
    return static_cast<float>(env->episode_length) * env->step_dt + time_range_[0];
  }
  return frozen_playback_time_;
}

bool State_WbcTracking::isPlaybackFinished() const
{
  if (!clip_playback_active_) {
    return false;
  }
  return currentPlaybackTime() >= time_range_[1] - env->step_dt * 0.5f;
}

void State_WbcTracking::handleClipSwitch()
{
  // Clip / getup / liedown UX lives on wbc_reference_node when using DDS.
  if (reference_source_ == "dds" || !clip_library_ || !FSMState::lowstate) {
    return;
  }

  const auto& joy = FSMState::lowstate->joystick;

  was_playback_finished_ = playback_finished_;
  playback_finished_ = isPlaybackFinished();

  if (playback_finished_ && !was_playback_finished_) {
    const auto kind = clip_library_->currentKind();
    if (kind == MotionClipLibrary::ClipKind::PoseGetup) {
      robot_is_up_ = true;
      enterClipSelectMode(time_range_[1]);
      return;
    }
    if (kind == MotionClipLibrary::ClipKind::PoseLiedown) {
      robot_is_up_ = false;
      pauseClipPlayback(time_range_[1]);
      spdlog::info("Liedown finished — robot is down");
      return;
    }
    if (kind == MotionClipLibrary::ClipKind::Browsable) {
      enterClipSelectMode(time_range_[1]);
      return;
    }
  }

  if (!robot_is_up_) {
    if (!clip_playback_active_ && clip_getup_fn_ && clip_getup_fn_(joy)) {
      if (clip_library_->selectPoseClip("getup")) {
        applyMotionLoader(true);
      }
    }
    return;
  }

  if (clip_liedown_fn_ && clip_liedown_fn_(joy)
      && (awaiting_clip_select_ || playback_finished_)) {
    if (clip_library_->selectPoseClip("liedown")) {
      applyMotionLoader(true);
    }
    return;
  }

  if (awaiting_clip_select_) {
    if (clip_next_fn_ && clip_next_fn_(joy)) {
      clip_library_->browseNextSelected();
      return;
    }
    if (clip_prev_fn_ && clip_prev_fn_(joy)) {
      clip_library_->browsePrevSelected();
      return;
    }
    if (clip_play_fn_ && clip_play_fn_(joy)) {
      if (clip_library_->activateSelectedBrowsable()) {
        applyMotionLoader(true);
      }
    }
  }
}

void State_WbcTracking::enter()
{
  for (size_t i = 0; i < env->robot->data.joint_stiffness.size(); ++i) {
    lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
    lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
    lowcmd->msg_.motor_cmd()[i].dq() = 0;
    lowcmd->msg_.motor_cmd()[i].tau() = 0;
  }

  if (reference_source_ == "dds") {
    // Policy tracks whatever wbc_reference_node publishes (standing hold until
    // a clip/Gen episode starts). Floor getup is also owned by the ref node.
    robot_is_up_ = true;
    awaiting_clip_select_ = false;
    clip_playback_active_ = true;
    frozen_playback_time_ = 0.0f;
    playback_finished_ = false;
    was_playback_finished_ = false;
    if (wbc_deploy::pendingWbcEntryMode() == wbc_deploy::WbcEntryMode::FromFloor) {
      spdlog::info(
        "WBC tracking (dds) from floor — press getup on wbc_reference_node "
        "(D-pad up) once the ref node is streaming");
      wbc_deploy::pendingWbcEntryMode() = wbc_deploy::WbcEntryMode::Standing;
    } else {
      spdlog::info(
        "WBC tracking (dds) — waiting for wbc_reference_node Arc on domain 101");
    }
  } else {
    motion = clip_library_->loader();
    if (wbc_deploy::pendingWbcEntryMode() == wbc_deploy::WbcEntryMode::FromFloor) {
      if (!clip_library_->selectPoseClip("getup")) {
        spdlog::error("Failed to load getup pose clip on WBC entry");
      } else {
        motion = clip_library_->loader();
      }
      robot_is_up_ = false;
      awaiting_clip_select_ = false;
      clip_playback_active_ = true;
      wbc_deploy::pendingWbcEntryMode() = wbc_deploy::WbcEntryMode::Standing;
      spdlog::info("WBC tracking started from floor — playing getup");
    } else {
      robot_is_up_ = true;
      enterClipSelectMode(0.0f);
      spdlog::info(
        "WBC tracking ready — selected clip: {}",
        clip_library_->selectedBrowsableName());
    }
  }

  syncPlaybackEndToMotion();
  if (clip_playback_active_) {
    playback_finished_ = false;
    was_playback_finished_ = false;
  }
  env->reset();
  policy_thread_running = true;
  policy_thread = std::thread([this] {
    using clock = std::chrono::high_resolution_clock;
    const auto dt = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(env->step_dt));

    auto sleep_till = clock::now() + dt;
    {
      std::lock_guard<std::mutex> lock(tracking_mtx_);
      if (reference_source_ == "clips" && clip_library_) {
        clip_library_->resetPlayback(env->robot->data, currentPlaybackTime());
      } else if (motion) {
        motion->reset(env->robot->data, currentPlaybackTime());
      }
    }

    while (policy_thread_running) {
      {
        std::lock_guard<std::mutex> lock(tracking_mtx_);
        if (motion && motion->consume_restart()) {
          env->reset();
          spdlog::info("WBC policy reset — new reference episode");
        }
        env->robot->update();
        motion->update(currentPlaybackTime());
        env->step();
      }
      std::this_thread::sleep_until(sleep_till);
      sleep_till += dt;
    }
  });
}

void State_WbcTracking::run()
{
  std::lock_guard<std::mutex> lock(tracking_mtx_);
  handleClipSwitch();

  const auto action = env->action_manager->processed_actions();
  std::vector<float> q_cmd(action.begin(), action.end());

  if (pd_torque_clip_.enabled && FSMState::lowstate) {
    std::vector<float> q_cur(q_cmd.size());
    std::vector<float> dq_cur(q_cmd.size());
    {
      std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
      for (size_t i = 0; i < q_cmd.size(); ++i) {
        const int motor = env->robot->data.joint_ids_map[i];
        q_cur[i] = FSMState::lowstate->msg_.motor_state()[motor].q();
        dq_cur[i] = FSMState::lowstate->msg_.motor_state()[motor].dq();
      }
    }
    wbc_deploy::clip_pd_torque_positions(
      q_cmd,
      q_cur,
      dq_cur,
      env->robot->data.joint_stiffness,
      env->robot->data.joint_damping,
      pd_torque_clip_);
  }

  for (size_t i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
    lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = q_cmd[i];
  }
}
