#include "service/motor/MotorControlService.hpp"

#include <algorithm>
#include <cmath>

#include "config/PIDConfig.hpp"
#include "config/RobotConfig.hpp"

namespace minicar::service {

MotorControlService::MotorControlService(
    driver::MotorEncoderDriver& left_driver,
    driver::MotorEncoderDriver& right_driver)
    : left_driver_(left_driver),
      right_driver_(right_driver),
      left_speed_pid_(config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD),
      right_speed_pid_(config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD)
{
    left_speed_pid_.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    right_speed_pid_.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    left_speed_pid_.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
    right_speed_pid_.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
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
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    left_velocity_mps_ = 0.0f;
    right_velocity_mps_ = 0.0f;
    left_ticks_ = 0;
    right_ticks_ = 0;
    resetControllers();
    left_driver_.stop();
    right_driver_.stop();
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
    left_velocity_mps_ = rpmToMps(left_driver_.getRPM());
    right_velocity_mps_ = rpmToMps(right_driver_.getRPM());
}

void MotorControlService::setVelocityTargets(float left_mps, float right_mps)
{
    left_target_mps_ = clampWheelSpeed(left_mps);
    right_target_mps_ = clampWheelSpeed(right_mps);
}

void MotorControlService::applyControl(float dt_s)
{
    if (!initialized_ || dt_s <= 0.0f) {
        left_driver_.stop();
        right_driver_.stop();
        return;
    }

    const float left_output = left_speed_pid_.update(left_target_mps_, left_velocity_mps_, dt_s);
    const float right_output = right_speed_pid_.update(right_target_mps_, right_velocity_mps_, dt_s);

    left_driver_.setOutput(left_output);
    right_driver_.setOutput(right_output);
}

void MotorControlService::stopMotor()
{
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    resetControllers();
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

void MotorControlService::resetControllers()
{
    left_speed_pid_.reset();
    right_speed_pid_.reset();
}

bool MotorControlService::isInitialized() const
{
    return initialized_;
}

float MotorControlService::rpmToMps(float rpm) const
{
    constexpr float PI = 3.14159265358979323846f;
    const float circumference_m = config::robot::WHEEL_DIAMETER_M * PI;
    return (rpm * circumference_m) / 60.0f;
}

float MotorControlService::clampWheelSpeed(float speed_mps) const
{
    return std::clamp(speed_mps, -config::robot::MAX_WHEEL_SPEED_MPS, config::robot::MAX_WHEEL_SPEED_MPS);
}

}  // namespace minicar::service
