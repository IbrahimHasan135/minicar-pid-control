#pragma once

namespace minicar::config::pid {

constexpr float SPEED_KP = 2.5f;
constexpr float SPEED_KI = 1.2f;
constexpr float SPEED_KD = 0.0f;
constexpr float SPEED_FEED_FORWARD = 1.5f;

constexpr float HEADING_KP = 0.02f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;

constexpr float SPEED_INTEGRAL_LIMIT = 0.8f;
constexpr float HEADING_INTEGRAL_LIMIT = 20.0f;

}  // namespace minicar::config::pid
