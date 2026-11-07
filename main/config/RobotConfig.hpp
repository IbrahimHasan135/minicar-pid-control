#pragma once

namespace minicar::config::robot {

constexpr float WHEEL_DIAMETER_M = 0.064f;
constexpr float WHEEL_BASE_M = 0.145f;
// Must match the quadrature count used by MotorEncoderDriver.
constexpr int ENCODER_TICKS_PER_REV = 1766;

constexpr float MAX_LINEAR_SPEED_MPS = 1.0f;
constexpr float MAX_WHEEL_SPEED_MPS = 1.2f;
constexpr float MAX_TURN_WHEEL_SPEED_MPS = 0.45f;
constexpr float STRAIGHT_HEADING_TRIGGER_DEG = 1.0f;
constexpr float STRAIGHT_RIGHT_WHEEL_TRIM_MPS = 0.06f;
// Disabled during IMU consistency diagnostics. The right motor only follows
// the measured speed of the left motor while this is false.
constexpr bool ENABLE_STRAIGHT_IMU_CORRECTION = false;
constexpr uint32_t IMU_DIAGNOSTIC_MQTT_QUEUE_LENGTH = 64;
constexpr float OPEN_LOOP_TURN_OUTPUT = 0.35f;
constexpr float MIN_TURN_OUTPUT = 0.12f;
constexpr float TURN_SLOWDOWN_ANGLE_DEG = 35.0f;
constexpr float MAX_MOTOR_OUTPUT = 1.0f;
constexpr float MIN_MOTOR_OUTPUT = -1.0f;

constexpr float MOVE_DISTANCE_TOLERANCE_M = 0.002f;
constexpr float MOVE_COMPLETE_SPEED_TOLERANCE_MPS = 0.03f;
constexpr float TURN_ANGLE_TOLERANCE_DEG = 2.0f;
constexpr float TURN_YAW_RATE_TOLERANCE_DPS = 5.0f;

}  // namespace minicar::config::robot
