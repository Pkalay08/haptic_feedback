#ifndef HAPTIC_FEEDBACK_EMG_SIGNAL_PROCESSOR_HPP
#define HAPTIC_FEEDBACK_EMG_SIGNAL_PROCESSOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "ADS1299.hpp"

namespace haptic_feedback {

class EMGSignalProcessor {
public:
    static constexpr std::size_t kMaxChannels = ADS1299::kMaxChannels;

    enum class RectificationMode : std::uint8_t {
        FullWave,
        HalfWave,
    };

    struct FilterConfig {
        float sample_rate_hz;
        float high_pass_cutoff_hz;
        float band_pass_low_cutoff_hz;
        float band_pass_high_cutoff_hz;
        float envelope_cutoff_hz;
        RectificationMode rectification_mode;
    };

    struct ADS1299Scale {
        float reference_voltage;
        float gain;
        std::int32_t max_positive_count;
    };

    struct ChannelSample {
        float raw_voltage;
        float high_passed;
        float band_passed;
        float rectified;
        float envelope;
    };

    struct Frame {
        std::array<ChannelSample, kMaxChannels> channels;
    };

    struct BiquadCoefficients {
        float b0;
        float b1;
        float b2;
        float a1;
        float a2;
    };

    struct BiquadState {
        float x1;
        float x2;
        float y1;
        float y2;
    };

    struct ChannelState {
        BiquadState high_pass;
        BiquadState band_pass;
        float envelope;
    };

    EMGSignalProcessor();
    explicit EMGSignalProcessor(const FilterConfig& filter_config);

    bool configure(const FilterConfig& filter_config);
    void reset();
    void resetChannel(std::size_t channel);

    ChannelSample processSample(std::size_t channel, std::int32_t adc_count);
    bool processFrame(const ADS1299::SampleFrame& input_frame, Frame& output_frame);

    float adcCountToVoltage(std::int32_t adc_count) const;
    float rectify(float value) const;

    const FilterConfig& filterConfig() const;
    void setScale(const ADS1299Scale& scale);
    const ADS1299Scale& scale() const;

    static FilterConfig defaultFilterConfig(float sample_rate_hz);
    static ADS1299Scale defaultADS1299Scale(float reference_voltage, float gain);

private:
    static constexpr float kDefaultEnvelopeCutoffHz = 5.0f;
    static constexpr float kDefaultHighPassCutoffHz = 20.0f;
    static constexpr float kDefaultBandPassLowCutoffHz = 20.0f;
    static constexpr float kDefaultBandPassHighCutoffHz = 450.0f;
    static constexpr std::int32_t kADS1299MaxPositiveCount = 0x7FFFFF;

    static BiquadCoefficients makeHighPass(float sample_rate_hz, float cutoff_hz);
    static BiquadCoefficients makeBandPass(float sample_rate_hz,
                                           float low_cutoff_hz,
                                           float high_cutoff_hz);
    static float processBiquad(const BiquadCoefficients& coefficients,
                               BiquadState& state,
                               float input);
    static bool validConfig(const FilterConfig& filter_config);

    FilterConfig filter_config_;
    ADS1299Scale scale_;
    BiquadCoefficients high_pass_coefficients_;
    BiquadCoefficients band_pass_coefficients_;
    float envelope_alpha_;
    std::array<ChannelState, kMaxChannels> channel_states_;
};

} // namespace haptic_feedback

#endif // HAPTIC_FEEDBACK_EMG_SIGNAL_PROCESSOR_HPP
