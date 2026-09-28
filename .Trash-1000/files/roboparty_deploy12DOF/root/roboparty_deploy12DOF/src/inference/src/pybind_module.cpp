// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include "robot_interface.hpp"

namespace py = pybind11;

class OnnxPolicyRunner {
   public:
    OnnxPolicyRunner(const std::string& model_path, int intra_threads = 1)
        : env_(ORT_LOGGING_LEVEL_WARNING, "OneEncosOnnxPolicy") {
        Ort::SessionOptions session_options;
        if (intra_threads > 0) {
            session_options.SetIntraOpNumThreads(intra_threads);
        }
        session_options.EnableCpuMemArena();
        session_options.EnableMemPattern();
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options);

        const size_t num_inputs = session_->GetInputCount();
        if (num_inputs != 1) {
            throw std::runtime_error("Only single-input ONNX models are supported: " + model_path);
        }
        input_names_.reserve(num_inputs);
        input_names_raw_.reserve(num_inputs);
        for (size_t i = 0; i < num_inputs; ++i) {
            Ort::AllocatedStringPtr input_name = session_->GetInputNameAllocated(i, allocator_);
            input_names_.push_back(input_name.get());
            auto type_info = session_->GetInputTypeInfo(i);
            input_shape_ = type_info.GetTensorTypeAndShapeInfo().GetShape();
        }

        const size_t num_outputs = session_->GetOutputCount();
        if (num_outputs != 1) {
            throw std::runtime_error("Only single-output ONNX models are supported: " + model_path);
        }
        output_names_.reserve(num_outputs);
        output_names_raw_.reserve(num_outputs);
        for (size_t i = 0; i < num_outputs; ++i) {
            Ort::AllocatedStringPtr output_name = session_->GetOutputNameAllocated(i, allocator_);
            output_names_.push_back(output_name.get());
            auto type_info = session_->GetOutputTypeInfo(i);
            output_shape_ = type_info.GetTensorTypeAndShapeInfo().GetShape();
        }

        normalize_shape(input_shape_);
        normalize_shape(output_shape_);
        input_size_ = shape_size(input_shape_);
        output_size_ = shape_size(output_shape_);
        if (input_size_ == 0 || output_size_ == 0) {
            throw std::runtime_error("Invalid ONNX tensor shape: " + model_path);
        }

        for (const auto& name : input_names_) {
            input_names_raw_.push_back(name.c_str());
        }
        for (const auto& name : output_names_) {
            output_names_raw_.push_back(name.c_str());
        }
    }

    std::vector<float> run(const std::vector<float>& input) {
        if (input.size() != input_size_) {
            throw std::runtime_error("ONNX input size mismatch: got " + std::to_string(input.size()) +
                                     ", expected " + std::to_string(input_size_));
        }
        std::vector<float> input_buffer(input.begin(), input.end());
        std::vector<float> output_buffer(output_size_, 0.0f);
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_buffer.data(), input_buffer.size(), input_shape_.data(), input_shape_.size());
        Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
            memory_info, output_buffer.data(), output_buffer.size(), output_shape_.data(), output_shape_.size());

        session_->Run(Ort::RunOptions{nullptr},
                      input_names_raw_.data(), &input_tensor, input_names_raw_.size(),
                      output_names_raw_.data(), &output_tensor, output_names_raw_.size());
        return output_buffer;
    }

    size_t input_size() const { return input_size_; }
    size_t output_size() const { return output_size_; }
    std::vector<int64_t> input_shape() const { return input_shape_; }
    std::vector<int64_t> output_shape() const { return output_shape_; }

   private:
    static void normalize_shape(std::vector<int64_t>& shape) {
        for (int64_t& dim : shape) {
            if (dim < 0) {
                dim = 1;
            }
        }
    }

    static size_t shape_size(const std::vector<int64_t>& shape) {
        if (shape.empty()) {
            return 0;
        }
        size_t size = 1;
        for (const int64_t dim : shape) {
            if (dim <= 0) {
                return 0;
            }
            size *= static_cast<size_t>(dim);
        }
        return size;
    }

    Ort::Env env_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_names_raw_;
    std::vector<const char*> output_names_raw_;
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;
    size_t input_size_{0};
    size_t output_size_{0};
};

PYBIND11_MODULE(robot_py, m) {
    py::class_<RobotInterface>(m, "RobotInterface")
        .def(py::init<const std::string&>(), py::arg("config_file"))
        .def("apply_action", &RobotInterface::apply_action,
             py::arg("p"),
             py::arg("v")   = std::vector<float>{},
             py::arg("kp")  = std::vector<float>{},
             py::arg("kd")  = std::vector<float>{},
             py::arg("tau") = std::vector<float>{})
        .def("init_motors", &RobotInterface::init_motors)
        .def("deinit_motors", &RobotInterface::deinit_motors)
        .def("reset_joints", &RobotInterface::reset_joints, py::arg("joint_default_angle"))
        .def("set_zeros", &RobotInterface::set_zeros)
        .def("clear_errors", &RobotInterface::clear_errors)
        .def("refresh_joints", &RobotInterface::refresh_joints)
        .def("get_joint_q", &RobotInterface::get_joint_q)
        .def("get_joint_vel", &RobotInterface::get_joint_vel)
        .def("get_joint_tau", &RobotInterface::get_joint_tau)
        .def("get_live_feedback_snapshot", &RobotInterface::get_live_feedback_snapshot)
        .def("get_quat", &RobotInterface::get_quat)
        .def("get_ang_vel", &RobotInterface::get_ang_vel)
        .def_property_readonly("is_init", [](const RobotInterface &r) {
            return r.is_init_.load();
        });

    py::class_<OnnxPolicyRunner>(m, "OnnxPolicyRunner")
        .def(py::init<const std::string&, int>(), py::arg("model_path"), py::arg("intra_threads") = 1)
        .def("run", &OnnxPolicyRunner::run, py::arg("input"))
        .def_property_readonly("input_size", &OnnxPolicyRunner::input_size)
        .def_property_readonly("output_size", &OnnxPolicyRunner::output_size)
        .def_property_readonly("input_shape", &OnnxPolicyRunner::input_shape)
        .def_property_readonly("output_shape", &OnnxPolicyRunner::output_shape);
}
