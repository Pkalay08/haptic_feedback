#include "CNNInference.hpp"
#include <cstring>

namespace arm_control {

EMGIMUInference::EMGIMUInference()
    : model_(nullptr), interpreter_(nullptr), is_ready_(false) {}

bool EMGIMUInference::LoadModel(const std::string& model_path) {
    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
        return false;
    }

    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model_, resolver);
    if (builder(&interpreter_) != kTfLiteOk || !interpreter_) {
        interpreter_.reset();
        return false;
    }

    if (interpreter_->AllocateTensors() != kTfLiteOk) {
        interpreter_.reset();
        return false;
    }

    is_ready_ = true;
    return true;
}

bool EMGIMUInference::SetInputTensor(int tensor_index, const float* data, size_t data_size) {
    if (!is_ready_ || !interpreter_) {
        return false;
    }
    if (tensor_index < 0 || tensor_index >= static_cast<int>(interpreter_->inputs().size())) {
        return false;
    }
    TfLiteTensor* tensor = interpreter_->input_tensor(tensor_index);
    if (!tensor || tensor->type != kTfLiteFloat32) {
        return false;
    }
    size_t expected = static_cast<size_t>(tensor->bytes) / sizeof(float);
    if (data_size != expected) {
        return false;
    }
    float* dest = interpreter_->typed_input_tensor<float>(tensor_index);
    if (!dest) {
        return false;
    }
    std::memcpy(dest, data, data_size * sizeof(float));
    return true;
}

bool EMGIMUInference::SetInput(const float* data, size_t data_size) {
    return SetInputTensor(0, data, data_size);
}

bool EMGIMUInference::RunInference() {
    if (!is_ready_ || !interpreter_) {
        return false;
    }
    return interpreter_->Invoke() == kTfLiteOk;
}

bool EMGIMUInference::GetOutputTensor(int tensor_index, float* output_buffer, size_t buffer_size) const {
    if (!is_ready_ || !interpreter_) {
        return false;
    }
    if (tensor_index < 0 || tensor_index >= static_cast<int>(interpreter_->outputs().size())) {
        return false;
    }
    TfLiteTensor* tensor = interpreter_->output_tensor(tensor_index);
    if (!tensor || tensor->type != kTfLiteFloat32) {
        return false;
    }
    size_t expected = static_cast<size_t>(tensor->bytes) / sizeof(float);
    if (buffer_size < expected) {
        return false;
    }
    const float* src = interpreter_->typed_output_tensor<float>(tensor_index);
    if (!src) {
        return false;
    }
    std::memcpy(output_buffer, src, expected * sizeof(float));
    return true;
}

bool EMGIMUInference::GetServoPositions(float* servo_positions, size_t num_servos) const {
    return GetOutputTensor(0, servo_positions, num_servos);
}

size_t EMGIMUInference::GetInputTensorSize(int tensor_index) const {
    if (!is_ready_ || !interpreter_) {
        return 0;
    }
    if (tensor_index < 0 || tensor_index >= static_cast<int>(interpreter_->inputs().size())) {
        return 0;
    }
    TfLiteTensor* tensor = interpreter_->input_tensor(tensor_index);
    if (!tensor || tensor->type != kTfLiteFloat32) {
        return 0;
    }
    return static_cast<size_t>(tensor->bytes) / sizeof(float);
}

size_t EMGIMUInference::GetOutputTensorSize(int tensor_index) const {
    if (!is_ready_ || !interpreter_) {
        return 0;
    }
    if (tensor_index < 0 || tensor_index >= static_cast<int>(interpreter_->outputs().size())) {
        return 0;
    }
    TfLiteTensor* tensor = interpreter_->output_tensor(tensor_index);
    if (!tensor || tensor->type != kTfLiteFloat32) {
        return 0;
    }
    return static_cast<size_t>(tensor->bytes) / sizeof(float);
}

int EMGIMUInference::GetNumInputTensors() const {
    return is_ready_ && interpreter_ ? static_cast<int>(interpreter_->inputs().size()) : 0;
}

int EMGIMUInference::GetNumOutputTensors() const {
    return is_ready_ && interpreter_ ? static_cast<int>(interpreter_->outputs().size()) : 0;
}

bool EMGIMUInference::IsReady() const {
    return is_ready_;
}

} // namespace arm_control