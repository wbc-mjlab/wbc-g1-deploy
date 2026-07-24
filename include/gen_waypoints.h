#pragma once

#include <vector>

namespace wbc_deploy {

/// Constant body-frame ``[vx, vy, wz]`` → sparse waypoint features (xy then angle).
/// Matches ``wbc_gen.play.waypoint_ctrl.integrate_vel_to_sparse_waypoints``.
void integrate_vel_to_sparse_waypoints(
  float vx,
  float vy,
  float wz,
  const std::vector<int>& horizons,
  float dt,
  float position_scale,
  std::vector<float>& xy_out,
  std::vector<float>& ang_out);

/// Optional low-pass on height waypoints: far = intention, nearer lags.
/// ``tau <= 0`` snaps every horizon to ``height_cmd``.
void lowpass_height_waypoints(
  std::vector<float>& height_wp,
  float height_cmd,
  float dt,
  const std::vector<int>& horizons,
  float tau);

}  // namespace wbc_deploy
