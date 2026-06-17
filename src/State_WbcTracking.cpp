#include "State_WbcTracking.h"

#include "MotionClipLibrary.h"
#include "joystick_expr.h"
#include "unitree_articulation.h"
#include "wbc_entry_mode.h"
#include "wbc_tracking_params.h"
#include "isaaclab/algorithms/algorithms.h"

#include <spdlog/spdlog.h>

#include <unordered_map>

std::shared_ptr<WbcMotionLoader> State_WbcTracking::motion = nullptr;

State_WbcTracking::~State_WbcTracking()
{
  exit();
}

State_WbcTracking::State_WbcTracking(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
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

  std::filesystem::path clips_dir = param::config_string(
    cfg, "clips_dir", param::config_string(root, "clips_dir", "config/clips"));
  clips_dir = param::resolve_path_under_proj(clips_dir);

  std::filesystem::path manifest = param::config_string(
    cfg, "clips_manifest", param::config_string(root, "clips_manifest", "manifest.yaml"));
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

  auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
    FSMState::lowstate);

  env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
  env->alg = std::make_unique<isaaclab::OrtRunner>(
    wbc_deploy::resolve_onnx_path(policy_dir).string());

  const std::string clip_next_expr = cfg["clip_next"].as<std::string>("RT + right.on_pressed");
  const std::string clip_prev_expr = cfg["clip_prev"].as<std::string>("RT + left.on_pressed");
  const std::string clip_getup_expr = cfg["clip_getup"].as<std::string>("up.on_pressed");
  const std::string clip_liedown_expr = cfg["clip_liedown"].as<std::string>("down.on_pressed");
  clip_next_fn_ = compileJoystickExpr(clip_next_expr);
  clip_prev_fn_ = compileJoystickExpr(clip_prev_expr);
  clip_getup_fn_ = compileJoystickExpr(clip_getup_expr);
  clip_liedown_fn_ = compileJoystickExpr(clip_liedown_expr);

  time_range_[0] = cfg["time_start"] ? cfg["time_start"].as<float>() : 0.0f;
  syncPlaybackEndToMotion();
}

void State_WbcTracking::syncPlaybackEndToMotion()
{
  if (!motion) {
    return;
  }
  time_range_[1] = motion->duration;
}

void State_WbcTracking::applyMotionLoader(bool reset_env)
{
  motion = clip_library_->loader();
  syncPlaybackEndToMotion();
  playback_finished_ = false;
  was_playback_finished_ = false;
  if (reset_env) {
    env->reset();
    clip_library_->resetPlayback(env->robot->data, time_range_[0]);
  }
  spdlog::info("Switched motion clip to: {}", clip_library_->currentName());
}

bool State_WbcTracking::isPlaybackFinished() const
{
  const float t = static_cast<float>(env->episode_length) * env->step_dt + time_range_[0];
  return t >= time_range_[1] - env->step_dt * 0.5f;
}

void State_WbcTracking::handleClipSwitch()
{
  if (!clip_library_ || !FSMState::lowstate) {
    return;
  }

  const auto& joy = FSMState::lowstate->joystick;
  bool switched = false;

  was_playback_finished_ = playback_finished_;
  playback_finished_ = isPlaybackFinished();

  if (playback_finished_ && !was_playback_finished_) {
    const auto kind = clip_library_->currentKind();
    if (kind == MotionClipLibrary::ClipKind::PoseGetup) {
      robot_is_up_ = true;
      awaiting_clip_select_ = true;
      spdlog::info("Getup finished — select a clip with RT + D-pad left/right");
      return;
    }
    if (kind == MotionClipLibrary::ClipKind::PoseLiedown) {
      robot_is_up_ = false;
      spdlog::info("Liedown finished — robot is down");
    }
  }

  if (awaiting_clip_select_) {
    if (clip_next_fn_ && clip_next_fn_(joy)) {
      switched = clip_library_->nextBrowsableClip();
    } else if (clip_prev_fn_ && clip_prev_fn_(joy)) {
      switched = clip_library_->prevBrowsableClip();
    }
    if (switched) {
      awaiting_clip_select_ = false;
      applyMotionLoader(true);
    }
    return;
  }

  if (!robot_is_up_) {
    if (playback_finished_ && clip_getup_fn_ && clip_getup_fn_(joy)) {
      switched = clip_library_->selectPoseClip("getup");
    }
  } else if (playback_finished_ && clip_liedown_fn_ && clip_liedown_fn_(joy)) {
    switched = clip_library_->selectPoseClip("liedown");
  } else if (clip_library_->currentKind() == MotionClipLibrary::ClipKind::Browsable) {
    if (clip_next_fn_ && clip_next_fn_(joy)) {
      switched = clip_library_->nextBrowsableClip();
    } else if (clip_prev_fn_ && clip_prev_fn_(joy)) {
      switched = clip_library_->prevBrowsableClip();
    }
  }

  if (!switched) {
    return;
  }

  applyMotionLoader(true);
}

void State_WbcTracking::enter()
{
  for (size_t i = 0; i < env->robot->data.joint_stiffness.size(); ++i) {
    lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
    lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
    lowcmd->msg_.motor_cmd()[i].dq() = 0;
    lowcmd->msg_.motor_cmd()[i].tau() = 0;
  }

  motion = clip_library_->loader();
  if (wbc_deploy::pendingWbcEntryMode() == wbc_deploy::WbcEntryMode::FromFloor) {
    if (!clip_library_->selectPoseClip("getup")) {
      spdlog::error("Failed to load getup pose clip on WBC entry");
    } else {
      motion = clip_library_->loader();
    }
    robot_is_up_ = false;
    awaiting_clip_select_ = false;
    wbc_deploy::pendingWbcEntryMode() = wbc_deploy::WbcEntryMode::Standing;
    spdlog::info("WBC tracking started from floor — playing getup");
  } else {
    robot_is_up_ = true;
    awaiting_clip_select_ = false;
  }

  syncPlaybackEndToMotion();
  playback_finished_ = false;
  was_playback_finished_ = false;
  env->reset();
  policy_thread_running = true;
  policy_thread = std::thread([this] {
    using clock = std::chrono::high_resolution_clock;
    const auto dt = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(env->step_dt));

    auto sleep_till = clock::now() + dt;
    clip_library_->resetPlayback(env->robot->data, time_range_[0]);

    while (policy_thread_running) {
      {
        std::lock_guard<std::mutex> lock(tracking_mtx_);
        env->robot->update();
        motion->update(env->episode_length * env->step_dt + time_range_[0]);
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
  for (size_t i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
    lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
  }
}
