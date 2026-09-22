#pragma once

namespace minicar::config::robot {

constexpr float WHEEL_DIAMETER_M = 0.065f;
constexpr float WHEEL_BASE_M = 0.145f;
constexpr int ENCODER_TICKS_PER_REV = 360;

constexpr float MAX_LINEAR_SPEED_MPS = 1.0f;
constexpr float MAX_WHEEL_SPEED_MPS = 1.2f;
constexpr float MAX_TURN_WHEEL_SPEED_MPS = 0.45f;
constexpr float MAX_MOTOR_OUTPUT = 1.0f;
constexpr float MIN_MOTOR_OUTPUT = -1.0f;

constexpr float MOVE_DISTANCE_TOLERANCE_M = 0.02f;
constexpr float MOVE_COMPLETE_SPEED_TOLERANCE_MPS = 0.03f;
constexpr float TURN_ANGLE_TOLERANCE_DEG = 2.0f;
constexpr float TURN_YAW_RATE_TOLERANCE_DPS = 5.0f;

}  // namespace minicar::config::robot
