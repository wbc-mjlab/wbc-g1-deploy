#include "gen_waypoints.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wbc_deploy {
namespace {

float exp_smooth(float current, float target, float dt, float tau)
{
  const float alpha = 1.0f - std::exp(-dt / std::max(tau, 1.0e-4f));
  return current + alpha * (target - current);
}

}  // namespace

void integrate_vel_to_sparse_waypoints(
  float vx,
  float vy,
  float wz,
  const std::vector<int>& horizons,
  float dt,
  float position_scale,
  std::vector<float>& xy_out,
  std::vector<float>& ang_out)
{
  if (position_scale <= 0.0f) {
    throw std::invalid_argument("position_scale must be positive");
  }
  const int k = static_cast<int>(horizons.size());
  xy_out.assign(static_cast<size_t>(k * 2), 0.0f);
  ang_out.assign(static_cast<size_t>(k * 2), 0.0f);
  if (k == 0) {
    return;
  }

  int max_h = 0;
  for (int h : horizons) {
    max_h = std::max(max_h, h);
  }
  std::vector<float> px(static_cast<size_t>(max_h + 1), 0.0f);
  std::vector<float> py(static_cast<size_t>(max_h + 1), 0.0f);
  std::vector<float> psi(static_cast<size_t>(max_h + 1), 0.0f);

  for (int step = 0; step < max_h; ++step) {
    const float c = std::cos(psi[static_cast<size_t>(step)]);
    const float s = std::sin(psi[static_cast<size_t>(step)]);
    px[static_cast<size_t>(step + 1)] =
      px[static_cast<size_t>(step)] + (c * vx - s * vy) * dt;
    py[static_cast<size_t>(step + 1)] =
      py[static_cast<size_t>(step)] + (s * vx + c * vy) * dt;
    psi[static_cast<size_t>(step + 1)] = psi[static_cast<size_t>(step)] + wz * dt;
  }

  for (int i = 0; i < k; ++i) {
    const int nf = horizons[static_cast<size_t>(i)];
    xy_out[static_cast<size_t>(2 * i)] = px[static_cast<size_t>(nf)] / position_scale;
    xy_out[static_cast<size_t>(2 * i + 1)] = py[static_cast<size_t>(nf)] / position_scale;
    const float dpsi = psi[static_cast<size_t>(nf)];
    ang_out[static_cast<size_t>(2 * i)] = std::cos(dpsi);
    ang_out[static_cast<size_t>(2 * i + 1)] = std::sin(dpsi);
  }
}

void lowpass_height_waypoints(
  std::vector<float>& height_wp,
  float height_cmd,
  float dt,
  const std::vector<int>& horizons,
  float tau)
{
  const int k = static_cast<int>(horizons.size());
  if (static_cast<int>(height_wp.size()) != k) {
    throw std::invalid_argument("height_wp size must match horizons");
  }
  if (k == 0) {
    return;
  }
  if (tau <= 0.0f) {
    std::fill(height_wp.begin(), height_wp.end(), height_cmd);
    return;
  }
  // Far = intention; nearer lags toward farther.
  height_wp[static_cast<size_t>(k - 1)] = exp_smooth(
    height_wp[static_cast<size_t>(k - 1)], height_cmd, dt, tau);
  for (int i = k - 2; i >= 0; --i) {
    height_wp[static_cast<size_t>(i)] = exp_smooth(
      height_wp[static_cast<size_t>(i)],
      height_wp[static_cast<size_t>(i + 1)],
      dt,
      tau);
  }
}

}  // namespace wbc_deploy
