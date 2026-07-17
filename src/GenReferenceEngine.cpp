#include "GenReferenceEngine.h"

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <unordered_map>

namespace wbc_deploy {

GenReferenceEngine::GenReferenceEngine(
  const std::filesystem::path& params_dir,
  int infer_log_every)
: params_(load_gen_deploy_params(params_dir))
, obs_(params_)
{
  const auto onnx = resolve_generator_onnx(params_);
  runner_ = std::make_unique<isaaclab::OrtRunner>(
    onnx.string(), "gen", infer_log_every);
  spdlog::info(
    "Gen ONNX loaded: {} (infer_log_every={})",
    onnx.string(),
    infer_log_every);
}

void GenReferenceEngine::reset()
{
  obs_.reset(standing_proprio_sample(params_));
}

void GenReferenceEngine::push_proprio(const GenProprioSample& sample)
{
  obs_.push(sample);
}

void GenReferenceEngine::set_height_cmd(float height_m)
{
  obs_.set_height_cmd(height_m);
}

void GenReferenceEngine::seed_height(float height_m)
{
  obs_.seed_height(height_m);
}

std::vector<float> GenReferenceEngine::step(float vx, float vy, float wz)
{
  if (!obs_.history_ready()) {
    throw std::runtime_error("Gen history not ready");
  }
  auto flat = obs_.build_obs(vx, vy, wz);
  std::unordered_map<std::string, std::vector<float>> inputs;
  inputs[params_.model.onnx_input_name] = std::move(flat);
  auto out = runner_->act(inputs);
  if (static_cast<int>(out.size()) != params_.output_dim) {
    throw std::runtime_error(
      "Gen ONNX output dim " + std::to_string(out.size()) +
      " != expected " + std::to_string(params_.output_dim));
  }
  return out;
}

}  // namespace wbc_deploy
