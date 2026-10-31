#pragma once

namespace minicar::config::pid {

constexpr float SPEED_KP = 2.3f;
constexpr float SPEED_KI = 1.1f;
constexpr float SPEED_KD = 0.0f;
constexpr float SPEED_FEED_FORWARD = 1.55f;
constexpr float SPEED_FEEDBACK_FILTER_ALPHA = 0.35f;
constexpr float SPEED_OUTPUT_SLEW_RATE = 2.5f;

constexpr float SPEED_SYNC_KP = 0.6f;
constexpr float SPEED_SYNC_KI = 0.15f;
constexpr float SPEED_SYNC_KD = 0.0f;
constexpr float SPEED_SYNC_OUTPUT_LIMIT = 0.12f;
constexpr float SPEED_SYNC_INTEGRAL_LIMIT = 0.3f;

constexpr float HEADING_KP = 0.02f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;

constexpr float SPEED_INTEGRAL_LIMIT = 0.6f;
constexpr float HEADING_INTEGRAL_LIMIT = 20.0f;

}  // namespace minicar::config::pid
