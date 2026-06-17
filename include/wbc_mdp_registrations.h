#pragma once

namespace wbc_deploy {

/// Call once before constructing ManagerBasedRLEnv so static REGISTER_* in
/// wbc_mdp_registrations.cpp are linked from the static library.
void ensure_mdp_registered();

}  // namespace wbc_deploy
