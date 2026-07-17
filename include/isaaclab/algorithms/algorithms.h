// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

class Algorithms
{
public:
    virtual ~Algorithms() = default;
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }

    /// Wall time of the last ``act()`` call (ONNX Session::Run), milliseconds.
    virtual double last_infer_ms() const { return 0.0; }

    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    /// ``log_every_n``: print infer timing every N runs (0 = silent).
    explicit OrtRunner(
      std::string model_path,
      std::string label = "onnx",
      int log_every_n = 0)
    : label_(std::move(label))
    , log_every_n_(log_every_n > 0 ? log_every_n : 0)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for (int i(0); i < static_cast<int>(input_names.size()); ++i) {
            const std::string name_str(input_names[static_cast<size_t>(i)]);
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(
              memory_info,
              input_data.data(),
              input_sizes[static_cast<size_t>(i)],
              input_shapes[static_cast<size_t>(i)].data(),
              input_shapes[static_cast<size_t>(i)].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        const auto t0 = std::chrono::steady_clock::now();
        auto output_tensor = session->Run(
          Ort::RunOptions{nullptr},
          input_names.data(),
          input_tensors.data(),
          input_tensors.size(),
          output_names.data(),
          1);
        const auto t1 = std::chrono::steady_clock::now();
        last_infer_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
        ++infer_count_;
        infer_ms_sum_ += last_infer_ms_;
        if (last_infer_ms_ > infer_ms_max_) {
            infer_ms_max_ = last_infer_ms_;
        }
        if (log_every_n_ > 0 &&
            (infer_count_ == 1 ||
             infer_count_ % static_cast<uint64_t>(log_every_n_) == 0)) {
            spdlog::info(
              "{} infer: last={:.3f} ms  avg={:.3f} ms  max={:.3f} ms  n={}",
              label_,
              last_infer_ms_,
              infer_ms_sum_ / static_cast<double>(infer_count_),
              infer_ms_max_,
              infer_count_);
        }

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

    double last_infer_ms() const override { return last_infer_ms_; }

private:
    std::string label_;
    int log_every_n_ = 0;
    double last_infer_ms_ = 0.0;
    double infer_ms_sum_ = 0.0;
    double infer_ms_max_ = 0.0;
    uint64_t infer_count_ = 0;

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};
};
