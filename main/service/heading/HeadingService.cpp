#include "service/heading/HeadingService.hpp"

#include <algorithm>
#include <cmath>

#include "config/PIDConfig.hpp"
#include "config/RobotConfig.hpp"

namespace minicar::service {

HeadingService::HeadingService(driver::IMUDriver& imu)
    : imu_(imu), heading_pid_(config::pid::HEADING_KP, config::pid::HEADING_KI, config::pid::HEADING_KD)
{
    heading_pid_.setOutputLimit(-config::robot::MAX_TURN_WHEEL_SPEED_MPS, config::robot::MAX_TURN_WHEEL_SPEED_MPS);
    heading_pid_.setIntegralLimit(-config::pid::HEADING_INTEGRAL_LIMIT, config::pid::HEADING_INTEGRAL_LIMIT);
}

esp_err_t HeadingService::init()
{
    const esp_err_t result = imu_.init();
    initialized_ = (result == ESP_OK);
    return result;
}

void HeadingService::reset()
{
    heading_deg_ = 0.0f;
    yaw_rate_dps_ = 0.0f;
    resetController();
}

void HeadingService::updateHeading(float dt_s)
{
    if (!initialized_ || dt_s <= 0.0f || !imu_.isHealthy()) {
        yaw_rate_dps_ = 0.0f;
        return;
    }

    yaw_rate_dps_ = imu_.getYawRateDps();
    heading_deg_ = normalizeAngleDeg(heading_deg_ + (yaw_rate_dps_ * dt_s));
}

float HeadingService::getHeadingDeg() const
{
    return heading_deg_;
}

float HeadingService::getYawRateDps() const
{
    return yaw_rate_dps_;
}

float HeadingService::calculateHeadingCorrection(float target_heading_deg, float dt_s)
{
    const float error_deg = shortestAngleErrorDeg(target_heading_deg, heading_deg_);
    const float correction = heading_pid_.update(error_deg, 0.0f, dt_s);
    return std::clamp(correction, -config::robot::MAX_TURN_WHEEL_SPEED_MPS, config::robot::MAX_TURN_WHEEL_SPEED_MPS);
}

void HeadingService::resetHeadingReference()
{
    heading_deg_ = 0.0f;
    yaw_rate_dps_ = 0.0f;
    resetController();
}

void HeadingService::resetController()
{
    heading_pid_.reset();
}

bool HeadingService::isInitialized() const
{
    return initialized_;
}

float HeadingService::normalizeAngleDeg(float angle_deg)
{
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float HeadingService::shortestAngleErrorDeg(float target_deg, float current_deg)
{
    return normalizeAngleDeg(target_deg - current_deg);
}

}  // namespace minicar::service
