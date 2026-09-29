#pragma once

#include <cstdint>

#include "driver/pulse_cnt.h"
#include "esp_err.h"

namespace minicar::driver {

class MotorEncoderDriver {
public:
    esp_err_t init();

    void setOutput(float output);
    int32_t getTicks() const;
    float getRPM() const;

    void stop();

private:
    int32_t readTicks() const;

    pcnt_unit_handle_t encoder_unit_{nullptr};
    pcnt_channel_handle_t encoder_channel_a_{nullptr};
    pcnt_channel_handle_t encoder_channel_b_{nullptr};

    mutable int32_t last_rpm_ticks_{0};
    mutable int64_t last_rpm_time_us_{0};
    mutable float rpm_{0.0f};
    bool initialized_{false};
    float output_{0.0f};
};

}  // namespace minicar::driver
