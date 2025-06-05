// timing_analyzer.h
#pragma once

#include <cstdint>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

class TimingAnalyzer {
   public:
    struct PulseStats {
        uint32_t count{0};
        uint32_t sum{0};
        uint32_t min_value{UINT32_MAX};
        uint32_t max_value{0};

        uint32_t average() const { return count > 0 ? sum / count : 0; }
        uint32_t range() const { return max_value > min_value ? max_value - min_value : 0; }

        void update(uint32_t value) {
            count++;
            sum += value;
            min_value = std::min(min_value, value);
            max_value = std::max(max_value, value);
        }

        void reset() {
            count = 0;
            sum = 0;
            min_value = UINT32_MAX;
            max_value = 0;
        }
    };

    struct ThresholdSuggestion {
        uint32_t min_threshold;
        uint32_t max_threshold;
        bool valid;
    };

   private:
    static constexpr size_t SAMPLE_BUFFER_SIZE = 1000;
    static constexpr uint32_t MIN_SAMPLES_FOR_ANALYSIS = 10;
    static constexpr uint32_t SAFETY_MARGIN_US = 50;

    bool enabled_{false};
    uint32_t pulse_samples_[SAMPLE_BUFFER_SIZE];
    uint16_t sample_index_{0};

    PulseStats short_pulse_stats_;  // '1' bits
    PulseStats long_pulse_stats_;   // '0' bits
    uint32_t rejected_count_{0};

    // Current thresholds for classification
    uint32_t short_pulse_min_;
    uint32_t short_pulse_max_;
    uint32_t long_pulse_min_;
    uint32_t long_pulse_max_;

   public:
    TimingAnalyzer(uint32_t short_min, uint32_t short_max, uint32_t long_min, uint32_t long_max)
        : short_pulse_min_(short_min),
          short_pulse_max_(short_max),
          long_pulse_min_(long_min),
          long_pulse_max_(long_max) {}

    void enable(bool enable = true) {
        enabled_ = enable;
        if (enable) {
            reset();
        }
    }

    bool is_enabled() const { return enabled_; }

    void reset() {
        sample_index_ = 0;
        short_pulse_stats_.reset();
        long_pulse_stats_.reset();
        rejected_count_ = 0;
        memset(pulse_samples_, 0, sizeof(pulse_samples_));
    }

    void record_pulse(uint32_t pulse_width) {
        if (!enabled_)
            return;

        // Store in circular buffer
        pulse_samples_[sample_index_] = pulse_width;
        sample_index_ = (sample_index_ + 1) % SAMPLE_BUFFER_SIZE;

        // Classify and update stats
        if (is_short_pulse(pulse_width)) {
            short_pulse_stats_.update(pulse_width);
        } else if (is_long_pulse(pulse_width)) {
            long_pulse_stats_.update(pulse_width);
        } else {
            rejected_count_++;
        }
    }

    ThresholdSuggestion suggest_short_pulse_thresholds() const {
        if (short_pulse_stats_.count < MIN_SAMPLES_FOR_ANALYSIS) {
            return {0, 0, false};
        }

        uint32_t avg = short_pulse_stats_.average();
        uint32_t tolerance = (short_pulse_stats_.range() / 2) + SAFETY_MARGIN_US;

        return {
            .min_threshold = avg > tolerance ? avg - tolerance : 0, .max_threshold = avg + tolerance, .valid = true};
    }

    ThresholdSuggestion suggest_long_pulse_thresholds() const {
        if (long_pulse_stats_.count < MIN_SAMPLES_FOR_ANALYSIS) {
            return {0, 0, false};
        }

        uint32_t avg = long_pulse_stats_.average();
        uint32_t tolerance = (long_pulse_stats_.range() / 2) + SAFETY_MARGIN_US;

        return {
            .min_threshold = avg > tolerance ? avg - tolerance : 0, .max_threshold = avg + tolerance, .valid = true};
    }

    void log_analysis(const char* tag = "timing_analyzer") const {
        if (!enabled_) {
            ESP_LOGW(tag, "Timing analysis not enabled");
            return;
        }

        ESP_LOGI(tag, "=== Timing Analysis Results ===");
        ESP_LOGI(tag, "CPU Frequency: %u MHz", getCpuFrequencyMhz());

        // Short pulse analysis
        if (short_pulse_stats_.count > 0) {
            ESP_LOGI(tag, "Short pulses ('1' bits): count=%u, avg=%uμs, range=%u-%uμs", short_pulse_stats_.count,
                     short_pulse_stats_.average(), short_pulse_stats_.min_value, short_pulse_stats_.max_value);

            auto suggestion = suggest_short_pulse_thresholds();
            if (suggestion.valid) {
                ESP_LOGI(tag, "  Suggested range: %u-%uμs (current: %u-%uμs)", suggestion.min_threshold,
                         suggestion.max_threshold, short_pulse_min_, short_pulse_max_);
            }
        }

        // Long pulse analysis
        if (long_pulse_stats_.count > 0) {
            ESP_LOGI(tag, "Long pulses ('0' bits): count=%u, avg=%uμs, range=%u-%uμs", long_pulse_stats_.count,
                     long_pulse_stats_.average(), long_pulse_stats_.min_value, long_pulse_stats_.max_value);

            auto suggestion = suggest_long_pulse_thresholds();
            if (suggestion.valid) {
                ESP_LOGI(tag, "  Suggested range: %u-%uμs (current: %u-%uμs)", suggestion.min_threshold,
                         suggestion.max_threshold, long_pulse_min_, long_pulse_max_);
            }
        }

        ESP_LOGI(tag, "Rejected pulses: %u", rejected_count_);
        ESP_LOGI(tag, "Total samples collected: %u",
                 std::min(sample_index_, static_cast<uint16_t>(SAMPLE_BUFFER_SIZE)));
    }

    // Getters for current stats
    const PulseStats& get_short_pulse_stats() const { return short_pulse_stats_; }
    const PulseStats& get_long_pulse_stats() const { return long_pulse_stats_; }
    uint32_t get_rejected_count() const { return rejected_count_; }

   private:
    bool is_short_pulse(uint32_t pulse_width) const {
        return pulse_width >= short_pulse_min_ && pulse_width <= short_pulse_max_;
    }

    bool is_long_pulse(uint32_t pulse_width) const {
        return pulse_width >= long_pulse_min_ && pulse_width <= long_pulse_max_;
    }
};

}  // namespace fineoffset
}  // namespace esphome
