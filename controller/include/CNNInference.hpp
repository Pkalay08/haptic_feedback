#ifndef CNN_INFERENCE_HPP
#define CNN_INFERENCE_HPP

#include <string>
#include <vector>
#include <cstdint>

// Forward declaration of LiteRT C API types
struct TfLiteModel;
struct TfLiteInterpreterOptions;
struct TfLiteInterpreter;

class CNNInference {
public:
    CNNInference(const std::string& model_path);
    ~CNNInference();

    // Run inference on a window of shape (window_size, num_features)
    // Returns a vector of 5 servo positions (normalized, multiply by label_max to recover)
    std::vector<float> predict(const std::vector<std::vector<float>>& input_window);

    // Check if the model loaded successfully
    bool isLoaded() const { return interpreter_ != nullptr; }

private:
    TfLiteModel* model_ = nullptr;
    TfLiteInterpreterOptions* options_ = nullptr;
    TfLiteInterpreter* interpreter_ = nullptr;

    int window_size_ = 50;
    int num_features_ = 13;
    int num_outputs_ = 5;
};

#endif // CNN_INFERENCE_HPP