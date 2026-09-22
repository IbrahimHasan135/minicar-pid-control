#pragma once

#include <cstdint>

#include "esp_err.h"

namespace minicar::driver {

class MotorEncoderDriver {
public:
    esp_err_t init();

    void setLeftOutput(float output);
    void setRightOutput(float output);

    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    float getLeftRPM() const;
    float getRightRPM() const;

    void stop();

private:
    float left_output_{0.0f};
    float right_output_{0.0f};
};

}  // namespace minicar::driver
