#include "service/motor/MotorControlService.hpp"

#include <cmath>

namespace minicar::service {

MotorControlService::MotorControlService(
    driver::MotorEncoderDriver& left_driver,
    driver::MotorEncoderDriver& right_driver)
    : left_driver_(left_driver), right_driver_(right_driver)
{
}

esp_err_t MotorControlService::init()
{
    const esp_err_t left_result = left_driver_.init();
    const esp_err_t right_result = right_driver_.init();

    initialized_ = (left_result == ESP_OK && right_result == ESP_OK);

    if (left_result != ESP_OK) {
        return left_result;
    }

    return right_result;
}

void MotorControlService::reset()
{
    left_velocity_mps_ = 0.0f;
    right_velocity_mps_ = 0.0f;
    left_ticks_ = 0;
    right_ticks_ = 0;
    stopMotor();
}

void MotorControlService::refreshFeedback()
{
    if (!initialized_) {
        left_velocity_mps_ = 0.0f;
        right_velocity_mps_ = 0.0f;
        left_ticks_ = 0;
        right_ticks_ = 0;
        return;
    }

    left_ticks_ = left_driver_.getTicks();
    right_ticks_ = right_driver_.getTicks();
    left_velocity_mps_ = left_driver_.getSpeedMps();
    right_velocity_mps_ = right_driver_.getSpeedMps();
}

void MotorControlService::setMotorOutputs(float left_output, float right_output)
{
    if (!initialized_) {
        stopMotor();
        return;
    }
    left_driver_.setOutput(left_output);
    right_driver_.setOutput(right_output);
}

void MotorControlService::stopMotor()
{
    left_driver_.stop();
    right_driver_.stop();
}

float MotorControlService::getLeftVelocityMps() const
{
    return left_velocity_mps_;
}

float MotorControlService::getRightVelocityMps() const
{
    return right_velocity_mps_;
}

float MotorControlService::getLeftRPM() const
{
    return left_driver_.getRPM();
}

float MotorControlService::getRightRPM() const
{
    return right_driver_.getRPM();
}

float MotorControlService::getAverageSpeedMps() const
{
    return (std::fabs(left_velocity_mps_) + std::fabs(right_velocity_mps_)) * 0.5f;
}

int32_t MotorControlService::getLeftTicks() const
{
    return left_ticks_;
}

int32_t MotorControlService::getRightTicks() const
{
    return right_ticks_;
}

bool MotorControlService::isInitialized() const
{
    return initialized_;
}

}  // namespace minicar::service
