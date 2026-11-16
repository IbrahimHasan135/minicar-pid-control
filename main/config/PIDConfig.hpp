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
// Ramp both wheel speed setpoints together to avoid one drivetrain breaking
// static friction before the other during launch.
constexpr float MOVE_TARGET_ACCELERATION_MPS2 = 0.25f;
// Recover accumulated left/right travel mismatch caused by launch transients.
// Units: speed correction (m/s) per meter of wheel-distance error.
constexpr float ENCODER_SYNC_KP_PER_S = 0.40f;
constexpr float ENCODER_SYNC_MAX_CORRECTION_MPS = 0.06f;
// The right drivetrain covers slightly less real distance per reported tick.
// Keep both encoder travel targets equal. The former right-wheel lead opposed
// the IMU correction and produced a small left drift during the last metre.
constexpr float RIGHT_ENCODER_DISTANCE_LEAD_RATIO = 0.0f;
// Left wheel is the reference. Both wheels receive the same ramped speed
// target, while the right target gets bounded encoder-position and optional
// heading corrections.
constexpr SpeedMaster SPEED_MASTER = SpeedMaster::LEFT;

constexpr float HEADING_KP = 0.02f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;

constexpr float SPEED_INTEGRAL_LIMIT = 0.6f;
constexpr float HEADING_INTEGRAL_LIMIT = 20.0f;

}  // namespace minicar::config::pid
