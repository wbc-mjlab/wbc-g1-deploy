#include "State_WbcTracking.h"
#include "MotionClipLibrary.h"
#include "g1_body_names.h"
#include "unitree_articulation.h"
#include "unitree_joystick_dsl.hpp"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/algorithms/algorithms.h"

#include <cmath>

namespace {

constexpr std::array<float, 3> kGravityW = {0.0f, 0.0f, -1.0f};

std::array<float, 3> quatApplyInverse(
  const Eigen::Quaternionf& q,
  const std::array<float, 3>& v)
{
  const Eigen::Vector3f out = q.conjugate() * Eigen::Vector3f(v[0], v[1], v[2]);
  return {out.x(), out.y(), out.z()};
}

int resolve_anchor_body_index(const cnpy::npz_t& npz, const std::string& anchor_name)
{
  auto match_names = [&](const auto& names) -> int {
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == anchor_name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  if (npz.find("body_names") != npz.end()) {
    const auto names_arr = npz.at("body_names");
    const size_t n = names_arr.shape[0];
    for (size_t i = 0; i < n; ++i) {
      std::string name(
        names_arr.data<char>() + i * names_arr.word_size,
        names_arr.word_size);
      while (!name.empty() && name.back() == '\0') {
        name.pop_back();
      }
      if (name == anchor_name) {
        return static_cast<int>(i);
      }
    }
  }

  const int fallback = match_names(G1_FULL_BODY_NAMES);
  if (fallback >= 0) {
    spdlog::warn(
      "NPZ missing body_names; using built-in G1 body list for anchor '{}'",
      anchor_name);
    return fallback;
  }

  spdlog::error("Anchor body '{}' not found in motion clip", anchor_name);
  return 0;
}

}  // namespace

WbcMotionLoader::WbcMotionLoader(
  const std::string& motion_file,
  const std::string& anchor_body_name,
  float step_dt)
: dt(step_dt)
{
  cnpy::npz_t npz = cnpy::npz_load(motion_file);
  anchor_body_index_ = resolve_anchor_body_index(npz, anchor_body_name);

  const auto body_pos_w = npz["body_pos_w"];
  const auto body_quat_w = npz["body_quat_w"];
  const auto body_lin_vel_w = npz["body_lin_vel_w"];
  const auto body_ang_vel_w = npz["body_ang_vel_w"];
  const auto joint_pos = npz["joint_pos"];

  const size_t num_frames_npz = body_pos_w.shape[0];
  const int num_bodies = static_cast<int>(body_pos_w.shape[1]);
  const int num_joints = static_cast<int>(joint_pos.shape[1]);
  const size_t body_stride_pos = static_cast<size_t>(num_bodies) * 3;
  const size_t body_stride_quat = static_cast<size_t>(num_bodies) * 4;
  const int ab = std::clamp(anchor_body_index_, 0, num_bodies - 1);

  for (size_t i = 0; i < num_frames_npz; ++i) {
    const float* pos_base = body_pos_w.data<float>() + i * body_stride_pos;
    anchor_positions_.emplace_back(
      pos_base[ab * 3 + 0], pos_base[ab * 3 + 1], pos_base[ab * 3 + 2]);

    const float* quat_base = body_quat_w.data<float>() + i * body_stride_quat;
    anchor_quaternions_.emplace_back(
      quat_base[ab * 4 + 0],
      quat_base[ab * 4 + 1],
      quat_base[ab * 4 + 2],
      quat_base[ab * 4 + 3]);

    const float* lin_base = body_lin_vel_w.data<float>() + i * body_stride_pos;
    anchor_lin_vels_.emplace_back(
      lin_base[ab * 3 + 0], lin_base[ab * 3 + 1], lin_base[ab * 3 + 2]);

    const float* ang_base = body_ang_vel_w.data<float>() + i * body_stride_pos;
    anchor_ang_vels_.emplace_back(
      ang_base[ab * 3 + 0], ang_base[ab * 3 + 1], ang_base[ab * 3 + 2]);

    Eigen::VectorXf jp(num_joints);
    for (int j = 0; j < num_joints; ++j) {
      jp[j] = joint_pos.data<float>()[i * num_joints + j];
    }
    dof_positions_.push_back(jp);
  }

  num_frames = static_cast<int>(num_frames_npz);
  duration = num_frames * dt;
  frame = 0;
  update(0.0f);
}

void WbcMotionLoader::update(float time)
{
  const float phase = std::clamp(time, 0.0f, duration);
  const float f = phase / dt;
  frame = std::min(static_cast<int>(std::floor(f)), num_frames - 1);
}

void WbcMotionLoader::reset(
  const isaaclab::ArticulationData& data,
  float t)
{
  update(t);
  const auto init_to_anchor = isaaclab::yawQuaternion(anchor_quat_w()).toRotationMatrix();
  const auto world_to_anchor = isaaclab::yawQuaternion(data.root_quat_w).toRotationMatrix();
  world_to_init_ = world_to_anchor * init_to_anchor.transpose();
}

Eigen::Vector3f WbcMotionLoader::anchor_pos_w() const
{
  return anchor_positions_[frame];
}

Eigen::Quaternionf WbcMotionLoader::anchor_quat_w() const
{
  return anchor_quaternions_[frame];
}

Eigen::Vector3f WbcMotionLoader::anchor_lin_vel_w() const
{
  return anchor_lin_vels_[frame];
}

Eigen::Vector3f WbcMotionLoader::anchor_ang_vel_w() const
{
  return anchor_ang_vels_[frame];
}

Eigen::VectorXf WbcMotionLoader::joint_pos() const
{
  return dof_positions_[frame];
}

std::vector<float> WbcMotionLoader::wbc_reference(float env_origin_z) const
{
  const Eigen::Quaternionf q = anchor_quat_w();
  const std::array<float, 3> lin_w = {
    anchor_lin_vel_w().x(), anchor_lin_vel_w().y(), anchor_lin_vel_w().z()};
  const std::array<float, 3> ang_w = {
    anchor_ang_vel_w().x(), anchor_ang_vel_w().y(), anchor_ang_vel_w().z()};
  const auto lin_b = quatApplyInverse(q, lin_w);
  const auto ang_b = quatApplyInverse(q, ang_w);
  const auto grav_b = quatApplyInverse(q, kGravityW);

  std::vector<float> cmd;
  cmd.reserve(10 + joint_pos().size());
  cmd.push_back(anchor_pos_w().z() - env_origin_z);
  cmd.insert(cmd.end(), lin_b.begin(), lin_b.end());
  cmd.insert(cmd.end(), ang_b.begin(), ang_b.end());
  cmd.insert(cmd.end(), grav_b.begin(), grav_b.end());
  const Eigen::VectorXf jp = joint_pos();
  for (int i = 0; i < jp.size(); ++i) {
    cmd.push_back(jp[i]);
  }
  return cmd;
}

std::shared_ptr<WbcMotionLoader> State_WbcTracking::motion = nullptr;


namespace isaaclab {
namespace mdp {

REGISTER_OBSERVATION(command)
{
  auto loader = State_WbcTracking::motion;
  const float z0 = env->cfg["wbc_tracking"]["env_origin_z"].as<float>(0.0f);
  return loader->wbc_reference(z0);
}

REGISTER_OBSERVATION(motion_anchor_pos_b)
{
  auto loader = State_WbcTracking::motion;
  const Eigen::Vector3f ref_p = loader->anchor_pos_w();
  const Eigen::Quaternionf ref_q = loader->anchor_quat_w();
  const Eigen::Vector3f rob_p = env->robot->data.root_pos_w;
  const Eigen::Quaternionf rob_q = env->robot->data.root_quat_w;

  const Eigen::Quaternionf rob_yaw = isaaclab::yawQuaternion(rob_q);
  const Eigen::Quaternionf ref_yaw = isaaclab::yawQuaternion(ref_q);
  Eigen::Vector3f delta = rob_yaw.conjugate() * (ref_p - rob_p);
  delta.z() = ref_p.z() - rob_p.z();
  return {delta.x(), delta.y(), delta.z()};
}

REGISTER_OBSERVATION(base_lin_vel)
{
  auto& v = env->robot->data.root_lin_vel_b;
  return std::vector<float>(v.data(), v.data() + v.size());
}

class ReferenceJointPositionAction : public JointAction
{
public:
  ReferenceJointPositionAction(YAML::Node cfg, ManagerBasedRLEnv* env)
  : JointAction(cfg, env) {}

  void process_actions(std::vector<float> actions) override
  {
    JointAction::process_actions(actions);
    auto loader = State_WbcTracking::motion;
    const Eigen::VectorXf q_ref = loader->joint_pos();
    for (int i = 0; i < _action_dim; ++i) {
      _processed_actions[i] += q_ref[i];
    }
  }
};

REGISTER_ACTION(ReferenceJointPositionAction);

}  // namespace mdp
}  // namespace isaaclab


namespace {

std::function<bool(const unitree::common::UnitreeJoystick&)> compileJoystickExpr(
  const std::string& expr)
{
  if (expr.empty()) {
    return {};
  }
  unitree::common::dsl::Parser parser(expr);
  const auto node = parser.Parse();
  return unitree::common::dsl::Compile(*node);
}

}  // namespace


State_WbcTracking::~State_WbcTracking()
{
  exit();
}

State_WbcTracking::State_WbcTracking(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
  auto cfg = param::config["FSM"][state_string];
  auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

  const auto deploy = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
  const float step_dt = deploy["step_dt"].as<float>();
  std::string anchor_body = "torso_link";
  if (deploy["wbc_tracking"] && deploy["wbc_tracking"]["anchor_body_name"]) {
    anchor_body = deploy["wbc_tracking"]["anchor_body_name"].as<std::string>();
  }

  std::filesystem::path clips_dir = cfg["clips_dir"].as<std::string>("config/clips");
  if (!clips_dir.is_absolute()) {
    clips_dir = param::proj_dir / clips_dir;
  }
  std::filesystem::path manifest = cfg["clips_manifest"].as<std::string>("manifest.yaml");
  if (!manifest.is_absolute()) {
    manifest = clips_dir / manifest;
  }

  clip_library_ = std::make_unique<MotionClipLibrary>(
    clips_dir, manifest, step_dt, anchor_body);
  motion = clip_library_->loader();

  auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);

  env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
    YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
    articulation);
  env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

  if (env->cfg["wbc_tracking"]) {
    env_origin_z_ = env->cfg["wbc_tracking"]["env_origin_z"].as<float>(0.0f);
    has_state_estimation_ = env->cfg["wbc_tracking"]["has_state_estimation"].as<bool>(false);
  }

  const std::string clip_next_expr = cfg["clip_next"].as<std::string>("RT + right.on_pressed");
  const std::string clip_prev_expr = cfg["clip_prev"].as<std::string>("RT + left.on_pressed");
  clip_next_fn_ = compileJoystickExpr(clip_next_expr);
  clip_prev_fn_ = compileJoystickExpr(clip_prev_expr);

  time_range_[0] = cfg["time_start"] ? cfg["time_start"].as<float>() : 0.0f;
  time_range_[1] = cfg["time_end"] ? cfg["time_end"].as<float>() : motion->duration;
}


void State_WbcTracking::handleClipSwitch()
{
  if (!clip_library_ || !FSMState::lowstate) {
    return;
  }
  const auto& joy = FSMState::lowstate->joystick;
  bool switched = false;
  if (clip_next_fn_ && clip_next_fn_(joy)) {
    switched = clip_library_->nextClip();
  } else if (clip_prev_fn_ && clip_prev_fn_(joy)) {
    switched = clip_library_->prevClip();
  }
  if (!switched) {
    return;
  }
  std::lock_guard<std::mutex> lock(tracking_mtx_);
  motion = clip_library_->loader();
  time_range_[1] = motion->duration;
  env->reset();
  clip_library_->resetPlayback(env->robot->data, time_range_[0]);
  spdlog::info("Switched motion clip to: {}", clip_library_->currentName());
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
  handleClipSwitch();

  std::lock_guard<std::mutex> lock(tracking_mtx_);
  const auto action = env->action_manager->processed_actions();
  for (size_t i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
    lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
  }
}
