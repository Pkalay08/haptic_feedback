// emg_imu_inference.hpp
#ifndef EMG_IMU_INFERENCE_HPP
#define EMG_IMU_INFERENCE_HPP

#include <cstddef>
#include <memory>
#include <string>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model_builder.h"
#include "tensorflow/lite/kernels/register.h"

namespace arm_control {

class EMGIMUInference {
public:
    EMGIMUInference();
    ~EMGIMUInference() = default;

    // Non-copyable, non-movable
    EMGIMUInference(const EMGIMUInference&) = delete;
    EMGIMUInference& operator=(const EMGIMUInference&) = delete;
    EMGIMUInference(EMGIMUInference&&) = delete;
    EMGIMUInference& operator=(EMGIMUInference&&) = delete;

    bool LoadModel(const std::string& model_path);

    bool SetInputTensor(int tensor_index, const float* data, size_t data_size);
    bool SetInput(const float* data, size_t data_size);
    bool RunInference();

    bool GetOutputTensor(int tensor_index, float* output_buffer, size_t buffer_size) const;
    bool GetServoPositions(float* servo_positions, size_t num_servos) const;

    size_t GetInputTensorSize(int tensor_index) const;
    size_t GetOutputTensorSize(int tensor_index) const;
    int GetNumInputTensors() const;
    int GetNumOutputTensors() const;
    bool IsReady() const;

private:
    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter> interpreter_;
    bool is_ready_;
};

} // namespace arm_control

#endif // EMG_IMU_INFERENCE_HPP