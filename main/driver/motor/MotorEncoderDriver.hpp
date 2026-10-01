#pragma once

#include <cstdint>

#include "config/MotorHardwareConfig.hpp"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

namespace minicar::driver {

class MotorEncoderDriver {
public:
    explicit MotorEncoderDriver(
        const config::motor_hardware::MotorPeripheralConfig& config =
            config::motor_hardware::LEFT_MOTOR);

    esp_err_t init();

    void setOutput(float output);
    int32_t getTicks() const;
    float getRPM() const;

    void stop();

private:
    bool hasPwm() const;
    bool hasEncoder() const;
    esp_err_t initPwm();
    esp_err_t initEncoder();
    esp_err_t writePwm(float output);
    int32_t readTicks() const;

    const config::motor_hardware::MotorPeripheralConfig& config_;
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
