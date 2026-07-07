#include "EMGSignalProcessor.hpp"

#include <cmath>

namespace haptic_feedback {
namespace {

constexpr float kPi = 3.14159265358979323846f;
float clampCutoff(float cutoff_hz, float sample_rate_hz)
{
    const float nyquist = sample_rate_hz * 0.5f;
    if (cutoff_hz <= 0.0f) {
        return 0.0f;
    }

    if (cutoff_hz >= nyquist) {
        return nyquist;
    }

    return cutoff_hz;
}

float absFloat(float value)
{
    return value < 0.0f ? -value : value;
}

} // namespace

EMGSignalProcessor::EMGSignalProcessor()
    : filter_config_(defaultFilterConfig(1000.0f)),
      scale_(defaultADS1299Scale(4.5f, 24.0f)),
      high_pass_coefficients_{},
      band_pass_coefficients_{},
      envelope_alpha_(0.0f),
      channel_states_{}
{
    configure(defaultFilterConfig(1000.0f));
}

EMGSignalProcessor::EMGSignalProcessor(const FilterConfig& filter_config)
    : filter_config_(defaultFilterConfig(1000.0f)),
      scale_(defaultADS1299Scale(4.5f, 24.0f)),
      high_pass_coefficients_{},
      band_pass_coefficients_{},
      envelope_alpha_(0.0f),
      channel_states_{}
{
    configure(filter_config);
}

bool EMGSignalProcessor::configure(const FilterConfig& filter_config)
{
    if (!validConfig(filter_config)) {
        return false;
    }

    filter_config_ = filter_config;
    high_pass_coefficients_ =
        makeHighPass(filter_config.sample_rate_hz,
                     filter_config.high_pass_cutoff_hz);
    band_pass_coefficients_ =
        makeBandPass(filter_config.sample_rate_hz,
                     filter_config.band_pass_low_cutoff_hz,
                     filter_config.band_pass_high_cutoff_hz);

    const float rc = 1.0f / (2.0f * kPi * filter_config.envelope_cutoff_hz);
    const float dt = 1.0f / filter_config.sample_rate_hz;
    envelope_alpha_ = dt / (rc + dt);

    reset();
    return true;
}

void EMGSignalProcessor::reset()
{
    for (std::size_t channel = 0; channel < kMaxChannels; ++channel) {
        resetChannel(channel);
    }
}

void EMGSignalProcessor::resetChannel(std::size_t channel)
{
    if (channel >= kMaxChannels) {
        return;
    }

    channel_states_[channel] = ChannelState{};
}

EMGSignalProcessor::ChannelSample
EMGSignalProcessor::processSample(std::size_t channel, std::int32_t adc_count)
{
    ChannelSample output = {};
    if (channel >= kMaxChannels) {
        return output;
    }

    ChannelState& state = channel_states_[channel];

    output.raw_voltage = adcCountToVoltage(adc_count);
    output.high_passed = processBiquad(high_pass_coefficients_,
                                       state.high_pass,
                                       output.raw_voltage);
    output.band_passed = processBiquad(band_pass_coefficients_,
                                       state.band_pass,
                                       output.high_passed);
    output.rectified = rectify(output.band_passed);
    state.envelope += envelope_alpha_ * (output.rectified - state.envelope);
    output.envelope = state.envelope;

    return output;
}

bool EMGSignalProcessor::processFrame(const ADS1299::SampleFrame& input_frame,
                                      Frame& output_frame)
{
    for (std::size_t channel = 0; channel < kMaxChannels; ++channel) {
        output_frame.channels[channel] =
            processSample(channel, input_frame.channels[channel]);
    }

    return true;
}

float EMGSignalProcessor::adcCountToVoltage(std::int32_t adc_count) const
{
    if (scale_.gain == 0.0f || scale_.max_positive_count == 0) {
        return 0.0f;
    }

    const float lsb_size =
        scale_.reference_voltage /
        (scale_.gain * static_cast<float>(scale_.max_positive_count));
    return static_cast<float>(adc_count) * lsb_size;
}

float EMGSignalProcessor::rectify(float value) const
{
    if (filter_config_.rectification_mode == RectificationMode::HalfWave) {
        return value > 0.0f ? value : 0.0f;
    }

    return absFloat(value);
}

const EMGSignalProcessor::FilterConfig& EMGSignalProcessor::filterConfig() const
{
    return filter_config_;
}

void EMGSignalProcessor::setScale(const ADS1299Scale& scale)
{
    if (scale.reference_voltage > 0.0f &&
        scale.gain > 0.0f &&
        scale.max_positive_count > 0) {
        scale_ = scale;
    }
}

const EMGSignalProcessor::ADS1299Scale& EMGSignalProcessor::scale() const
{
    return scale_;
}

EMGSignalProcessor::FilterConfig
EMGSignalProcessor::defaultFilterConfig(float sample_rate_hz)
{
    FilterConfig config = {};
    config.sample_rate_hz = sample_rate_hz;
    config.high_pass_cutoff_hz = kDefaultHighPassCutoffHz;
    config.band_pass_low_cutoff_hz = kDefaultBandPassLowCutoffHz;
    config.band_pass_high_cutoff_hz = kDefaultBandPassHighCutoffHz;
    config.envelope_cutoff_hz = kDefaultEnvelopeCutoffHz;
    config.rectification_mode = RectificationMode::FullWave;
    return config;
}

EMGSignalProcessor::ADS1299Scale
EMGSignalProcessor::defaultADS1299Scale(float reference_voltage, float gain)
{
    ADS1299Scale scale = {};
    scale.reference_voltage = reference_voltage;
    scale.gain = gain;
    scale.max_positive_count = kADS1299MaxPositiveCount;
    return scale;
}

EMGSignalProcessor::BiquadCoefficients
EMGSignalProcessor::makeHighPass(float sample_rate_hz, float cutoff_hz)
{
    cutoff_hz = clampCutoff(cutoff_hz, sample_rate_hz);
    if (cutoff_hz <= 0.0f || cutoff_hz >= (sample_rate_hz * 0.5f)) {
        return BiquadCoefficients{1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }

    const float omega = 2.0f * kPi * cutoff_hz / sample_rate_hz;
    const float sin_omega = std::sin(omega);
    const float cos_omega = std::cos(omega);
    const float q = 0.70710678118f;
    const float alpha = sin_omega / (2.0f * q);

    const float b0 = (1.0f + cos_omega) * 0.5f;
    const float b1 = -(1.0f + cos_omega);
    const float b2 = (1.0f + cos_omega) * 0.5f;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_omega;
    const float a2 = 1.0f - alpha;

    return BiquadCoefficients{
        b0 / a0,
        b1 / a0,
        b2 / a0,
        a1 / a0,
        a2 / a0,
    };
}

EMGSignalProcessor::BiquadCoefficients
EMGSignalProcessor::makeBandPass(float sample_rate_hz,
                                 float low_cutoff_hz,
                                 float high_cutoff_hz)
{
    low_cutoff_hz = clampCutoff(low_cutoff_hz, sample_rate_hz);
    high_cutoff_hz = clampCutoff(high_cutoff_hz, sample_rate_hz);
    if (low_cutoff_hz <= 0.0f ||
        high_cutoff_hz <= low_cutoff_hz ||
        high_cutoff_hz >= (sample_rate_hz * 0.5f)) {
        return BiquadCoefficients{1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }

    const float center_hz = std::sqrt(low_cutoff_hz * high_cutoff_hz);
    const float bandwidth_hz = high_cutoff_hz - low_cutoff_hz;
    const float q = center_hz / bandwidth_hz;
    const float omega = 2.0f * kPi * center_hz / sample_rate_hz;
    const float sin_omega = std::sin(omega);
    const float cos_omega = std::cos(omega);
    const float alpha = sin_omega / (2.0f * q);

    const float b0 = alpha;
    const float b1 = 0.0f;
    const float b2 = -alpha;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_omega;
    const float a2 = 1.0f - alpha;

    return BiquadCoefficients{
        b0 / a0,
        b1 / a0,
        b2 / a0,
        a1 / a0,
        a2 / a0,
    };
}

float EMGSignalProcessor::processBiquad(const BiquadCoefficients& coefficients,
                                        BiquadState& state,
                                        float input)
{
    const float output =
        coefficients.b0 * input +
        coefficients.b1 * state.x1 +
        coefficients.b2 * state.x2 -
        coefficients.a1 * state.y1 -
        coefficients.a2 * state.y2;

    state.x2 = state.x1;
    state.x1 = input;
    state.y2 = state.y1;
    state.y1 = output;

    return output;
}

bool EMGSignalProcessor::validConfig(const FilterConfig& filter_config)
{
    const float nyquist = filter_config.sample_rate_hz * 0.5f;

    return filter_config.sample_rate_hz > 0.0f &&
           filter_config.high_pass_cutoff_hz > 0.0f &&
           filter_config.band_pass_low_cutoff_hz > 0.0f &&
           filter_config.band_pass_high_cutoff_hz >
               filter_config.band_pass_low_cutoff_hz &&
           filter_config.band_pass_high_cutoff_hz < nyquist &&
           filter_config.envelope_cutoff_hz > 0.0f &&
           filter_config.envelope_cutoff_hz < nyquist;
}

} // namespace haptic_feedback
