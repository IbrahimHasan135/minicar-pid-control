#include "driver/motor/MotorEncoderDriver.hpp"

namespace minicar::driver {

esp_err_t MotorEncoderDriver::init()
{
    // TODO(Aranda): configure motor GPIO, LEDC/PWM, direction pins, and encoder counters.
    return ESP_ERR_NOT_SUPPORTED;
}

void MotorEncoderDriver::setLeftOutput(float output)
{
    // TODO(Aranda): apply left motor output to hardware.
    left_output_ = output;
}

void MotorEncoderDriver::setRightOutput(float output)
{
    // TODO(Aranda): apply right motor output to hardware.
    right_output_ = output;
}

int32_t MotorEncoderDriver::getLeftTicks() const
{
    // TODO(Aranda): return left encoder counter.
    return 0;
}

int32_t MotorEncoderDriver::getRightTicks() const
{
    // TODO(Aranda): return right encoder counter.
    return 0;
}

float MotorEncoderDriver::getLeftRPM() const
{
    // TODO(Aranda): return left wheel RPM from encoder feedback.
    return 0.0f;
}

float MotorEncoderDriver::getRightRPM() const
{
    // TODO(Aranda): return right wheel RPM from encoder feedback.
    return 0.0f;
}

void MotorEncoderDriver::stop()
{
    left_output_ = 0.0f;
    right_output_ = 0.0f;
    // TODO(Aranda): force both motor outputs to a safe stop state.
}

}  // namespace minicar::driver
