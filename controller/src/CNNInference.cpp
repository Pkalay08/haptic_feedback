#include "CNNInference.hpp"
#include <iostream>
#include <cstring>

// LiteRT C API headers (installed with the LiteRT C library)
extern "C" {
    #include "tensorflow/lite/c/c_api.h"
}

CNNInference::CNNInference(const std::string& model_path) {
    // 1. Load the model
    model_ = TfLiteModelCreateFromFile(model_path.c_str());
    if (!model_) {
        std::cerr << "Failed to load model: " << model_path << std::endl;
        return;
    }

    // 2. Create interpreter options
    options_ = TfLiteInterpreterOptionsCreate();

    // Optional: set number of threads (CM3 has 4 cores)
    TfLiteInterpreterOptionsSetNumThreads(options_, 4);

    // 3. Create the interpreter
    interpreter_ = TfLiteInterpreterCreate(model_, options_);
    if (!interpreter_) {
        std::cerr << "Failed to create interpreter" << std::endl;
        return;
    }

    // 4. Allocate tensors
    if (TfLiteInterpreterAllocateTensors(interpreter_) != kTfLiteOk) {
        std::cerr << "Failed to allocate tensors" << std::endl;
        return;
    }

    // 5. Verify input tensor shape
    const TfLiteTensor* input_tensor = TfLiteInterpreterGetInputTensor(interpreter_, 0);
    if (input_tensor) {
        int ndims = TfLiteTensorNumDims(input_tensor);
        if (ndims == 3) {
            window_size_ = TfLiteTensorDim(input_tensor, 1);
            num_features_ = TfLiteTensorDim(input_tensor, 2);
        }
    }

    std::cout << "Model loaded. Input shape: (" << window_size_ << ", " << num_features_ << ")" << std::endl;
}

CNNInference::~CNNInference() {
    if (interpreter_) TfLiteInterpreterDelete(interpreter_);
    if (options_) TfLiteInterpreterOptionsDelete(options_);
    if (model_) TfLiteModelDelete(model_);
}

std::vector<float> CNNInference::predict(const std::vector<std::vector<float>>& input_window) {
    std::vector<float> output(num_outputs_, 0.0f);

    if (!interpreter_) {
        std::cerr << "Interpreter not initialized" << std::endl;
        return output;
    }

    // 1. Get input tensor
    TfLiteTensor* input_tensor = TfLiteInterpreterGetInputTensor(interpreter_, 0);
    if (!input_tensor) return output;

    // 2. Copy input data into the tensor (flattened)
    float* input_data = static_cast<float*>(TfLiteTensorData(input_tensor));;
    if (!input_data) return output;

    size_t idx = 0;
    for (int i = 0; i < window_size_; i++) {
        for (int j = 0; j < num_features_; j++) {
            input_data[idx++] = input_window[i][j];
        }
    }

    // 3. Run inference
    if (TfLiteInterpreterInvoke(interpreter_) != kTfLiteOk) {
        std::cerr << "Inference failed" << std::endl;
        return output;
    }

    // 4. Get output tensor
    const TfLiteTensor* output_tensor = TfLiteInterpreterGetOutputTensor(interpreter_, 0);
    if (!output_tensor) return output;

    const float* output_data = static_cast<float*>(TfLiteTensorData(output_tensor));
    if (!output_data) return output;

    // 5. Copy output data
    for (int i = 0; i < num_outputs_; i++) {
        output[i] = output_data[i];
    }

    return output;
}