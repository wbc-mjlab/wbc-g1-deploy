#include "State_WbcTracking.h"

#include "motion_reference_obs.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

namespace isaaclab {
namespace mdp {

namespace {

float motion_env_origin_z(ManagerBasedRLEnv* env)
{
  return env->cfg["wbc_tracking"]["env_origin_z"].as<float>(0.0f);
}

#define REGISTER_MOTION_REF(name) \
  REGISTER_OBSERVATION(name) \
  { \
    return wbc_deploy::motion_reference_observation( \
      *State_WbcTracking::motion, #name, motion_env_origin_z(env)); \
  }

}  // namespace

REGISTER_MOTION_REF(ref_base_height)
REGISTER_MOTION_REF(ref_base_lin_vel_b)
REGISTER_MOTION_REF(ref_base_ang_vel_b)
REGISTER_MOTION_REF(ref_gravity_b)
REGISTER_MOTION_REF(ref_joint_pos)
REGISTER_MOTION_REF(ref_joint_vel)

REGISTER_OBSERVATION(command)
{
  return wbc_deploy::motion_reference_stack(
    *State_WbcTracking::motion, motion_env_origin_z(env));
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
