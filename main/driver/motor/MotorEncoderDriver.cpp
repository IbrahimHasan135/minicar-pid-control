#include "driver/motor/MotorEncoderDriver.hpp"

namespace minicar::driver {

esp_err_t MotorEncoderDriver::init()
{
    // TODO(Aranda): configure this motor GPIO, PWM, direction pins, and encoder counter.
    return ESP_ERR_NOT_SUPPORTED;
}

void MotorEncoderDriver::setOutput(float output)
{
    // TODO(Aranda): apply this motor output to hardware.
    output_ = output;
}

int32_t MotorEncoderDriver::getTicks() const
{
    // TODO(Aranda): return this motor encoder counter.
    return 0;
}

float MotorEncoderDriver::getRPM() const
{
    // TODO(Aranda): return this motor wheel RPM from encoder feedback.
    return 0.0f;
}

void MotorEncoderDriver::stop()
{
    output_ = 0.0f;
    // TODO(Aranda): force this motor output to a safe stop state.
}

}  // namespace minicar::driver
