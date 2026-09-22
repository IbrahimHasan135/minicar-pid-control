#include "service/control/PIDController.hpp"

#include <algorithm>
#include <cmath>

namespace minicar::service {

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
}

float PIDController::update(float setpoint, float measurement, float dt_s)
{
    if (!std::isfinite(setpoint) || !std::isfinite(measurement) || dt_s <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;
    integral_ = std::clamp(integral_ + (error * dt_s), min_integral_, max_integral_);

    float derivative = 0.0f;
    if (has_previous_error_) {
        derivative = (error - previous_error_) / dt_s;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    const float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
    return std::clamp(output, min_output_, max_output_);
}

void PIDController::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

void PIDController::setOutputLimit(float min_output, float max_output)
{
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::setIntegralLimit(float min_integral, float max_integral)
{
    min_integral_ = min_integral;
    max_integral_ = max_integral;
}

}  // namespace minicar::service
