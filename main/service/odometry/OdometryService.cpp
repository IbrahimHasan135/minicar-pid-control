#include "service/odometry/OdometryService.hpp"

#include <cmath>

#include "config/RobotConfig.hpp"

namespace minicar::service {

esp_err_t OdometryService::init()
{
    reset();
    return ESP_OK;
}

void OdometryService::reset()
{
    pose_ = model::Pose2D{};
    travelled_distance_m_ = 0.0f;
    last_left_ticks_ = 0;
    last_right_ticks_ = 0;
    has_last_ticks_ = false;
}

void OdometryService::updateOdometry(int32_t left_ticks, int32_t right_ticks, float heading_deg)
{
    if (!has_last_ticks_) {
        last_left_ticks_ = left_ticks;
        last_right_ticks_ = right_ticks;
        has_last_ticks_ = true;
        pose_.heading_deg = heading_deg;
        return;
    }

    const int32_t delta_left_ticks = left_ticks - last_left_ticks_;
    const int32_t delta_right_ticks = right_ticks - last_right_ticks_;

    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;

    const float left_distance_m = ticksToMeters(delta_left_ticks);
    const float right_distance_m = ticksToMeters(delta_right_ticks);
    const float delta_distance_m = (left_distance_m + right_distance_m) * 0.5f;

    constexpr float PI = 3.14159265358979323846f;
    const float heading_rad = heading_deg * (PI / 180.0f);

    pose_.x_m += delta_distance_m * std::cos(heading_rad);
    pose_.y_m += delta_distance_m * std::sin(heading_rad);
    pose_.heading_deg = heading_deg;
    travelled_distance_m_ += delta_distance_m;
}

float OdometryService::getTravelledDistanceM() const
{
    return travelled_distance_m_;
}

model::Pose2D OdometryService::getPose() const
{
    return pose_;
}

void OdometryService::resetDistanceReference()
{
    travelled_distance_m_ = 0.0f;
}

float OdometryService::ticksToMeters(int32_t ticks) const
{
    constexpr float PI = 3.14159265358979323846f;
    const float circumference_m = config::robot::WHEEL_DIAMETER_M * PI;
    return (static_cast<float>(ticks) / static_cast<float>(config::robot::ENCODER_TICKS_PER_REV)) * circumference_m;
}

}  // namespace minicar::service
