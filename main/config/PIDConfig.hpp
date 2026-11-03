#pragma once

namespace minicar::config::pid {

enum class SpeedMaster {
    INDEPENDENT,
    LEFT,
    RIGHT,
};

constexpr float SPEED_KP = 2.3f;
constexpr float SPEED_KI = 1.1f;
constexpr float SPEED_KD = 0.0f;
constexpr float SPEED_FEED_FORWARD = 1.55f;
constexpr float SPEED_FEEDBACK_FILTER_ALPHA = 0.35f;
constexpr float SPEED_OUTPUT_SLEW_RATE = 2.5f;
// Select the wheel with the more repeatable mechanical response as master.
// Change to RIGHT when the right motor is the better reference on another build.
constexpr SpeedMaster SPEED_MASTER = SpeedMaster::LEFT;

constexpr float HEADING_KP = 0.02f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;

constexpr float SPEED_INTEGRAL_LIMIT = 0.6f;
constexpr float HEADING_INTEGRAL_LIMIT = 20.0f;

}  // namespace minicar::config::pid
