#pragma once

#include <cstdint>

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
    float output_{0.0f};
};

}  // namespace minicar::driver
