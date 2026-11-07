#pragma once

namespace minicar::config::pid {

enum class SpeedMaster {
    INDEPENDENT,
    LEFT,
    RIGHT,
};

// Conservative high-response baseline for the 200 Hz motor control loop.
constexpr float SPEED_KP = 2.8f;
constexpr float SPEED_KI = 1.4f;
constexpr float SPEED_KD = 0.0f;
constexpr float SPEED_FEED_FORWARD = 1.55f;
constexpr float SPEED_FEEDBACK_FILTER_ALPHA = 0.35f;
constexpr float SPEED_OUTPUT_SLEW_RATE = 2.5f;
// Left wheel is the speed master; the right wheel normally follows its
// measured speed and is temporarily trimmed by the IMU heading supervisor.
constexpr SpeedMaster SPEED_MASTER = SpeedMaster::LEFT;

constexpr float HEADING_KP = 0.02f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;

constexpr float SPEED_INTEGRAL_LIMIT = 0.6f;
constexpr float HEADING_INTEGRAL_LIMIT = 20.0f;

}  // namespace minicar::config::pid
