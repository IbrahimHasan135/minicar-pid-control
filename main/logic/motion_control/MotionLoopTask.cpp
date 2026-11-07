#include "logic/motion_control/MotionLoopTask.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config/FreeRTOSConfig.hpp"
#include "config/PIDConfig.hpp"
#include "config/RobotConfig.hpp"
#include "esp_log.h"

namespace {
constexpr const char* kTag = "MotionLoopTask";
}

namespace minicar::logic::motion_control {

MotionLoopTask::MotionLoopTask(
    MotionControlContext& context,
    service::MotorControlService& motor_service,
    service::HeadingService& heading_service,
    service::OdometryService& odometry_service,
    QueueHandle_t telemetry_queue,
    logic::MqttPublishEventQueue& mqtt_publish_queue)
    : context_(context),
      motor_service_(motor_service),
      heading_service_(heading_service),
      odometry_service_(odometry_service),
      telemetry_queue_(telemetry_queue),
      mqtt_publish_queue_(mqtt_publish_queue),
      left_speed_pid_(config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD),
      right_speed_pid_(config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD)
{
    left_speed_pid_.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    right_speed_pid_.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    left_speed_pid_.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
    right_speed_pid_.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
}

BaseType_t MotionLoopTask::start()
{
    return xTaskCreate(
        &MotionLoopTask::taskEntry,
        "MotionLoopTask",
        config::rtos::CONTROL_TASK_STACK_WORDS,
        this,
        config::rtos::CONTROL_TASK_PRIORITY,
        nullptr);
}

void MotionLoopTask::taskEntry(void* task_context)
{
    static_cast<MotionLoopTask*>(task_context)->run();
}

void MotionLoopTask::run()
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    last_telemetry_tick_ = last_wake_tick;
    const TickType_t control_period_ticks = std::max<TickType_t>(
        1, pdMS_TO_TICKS(config::rtos::CONTROL_PERIOD_MS));

    while (true) {
        step(config::rtos::CONTROL_PERIOD_S);
        vTaskDelayUntil(&last_wake_tick, control_period_ticks);
    }
}

void MotionLoopTask::step(float dt_s)
{
    if (dt_s <= 0.0f || !std::isfinite(dt_s)) {
        failSafeStop();
        return;
    }

    if (context_.hasStopRequest()) {
        stopAndIdle();
        context_.clearStopRequest();
        return;
    }

    updateServices(dt_s);

    if (!servicesReady()) {
        failSafeStop();
        return;
    }

    startPendingCommand();
    handleSnapshot(context_.snapshot(), dt_s);
    applySpeedControl(dt_s);
    publishTelemetry();
    publishImuDiagnostic(context_.snapshot());
}

void MotionLoopTask::updateServices(float dt_s)
{
    heading_service_.updateHeading(dt_s);
    motor_service_.refreshFeedback();
    odometry_service_.updateOdometry(
        motor_service_.getLeftTicks(),
        motor_service_.getRightTicks(),
        heading_service_.getHeadingDeg());
}

void MotionLoopTask::startPendingCommand()
{
    model::MotionCommand command{};
    if (!context_.takePendingCommand(command)) {
        return;
    }

    switch (command.type) {
    case model::MotionType::MOVE_DISTANCE:
        startMove(command);
        break;
    case model::MotionType::TURN_ANGLE:
        startTurn(command);
        break;
    case model::MotionType::WAIT_SECONDS:
        startWait(command);
        break;
    case model::MotionType::STOP:
        stopAndIdle();
        break;
    }
}

void MotionLoopTask::startMove(const model::MotionCommand& command)
{
    if (!isValidMove(command)) {
        failSafeStop();
        return;
    }

    left_speed_pid_.reset();
    right_speed_pid_.reset();
    filtered_left_speed_mps_ = 0.0f;
    filtered_right_speed_mps_ = 0.0f;
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    feedback_filter_initialized_ = false;
    straight_heading_correction_ = StraightHeadingCorrection::NONE;
    heading_service_.resetController();

    const float speed_mps = std::clamp(
        std::fabs(command.speed),
        0.0f,
        config::robot::MAX_LINEAR_SPEED_MPS);

    // Encoder distance is the primary completion condition. Keep a generous
    // time limit only as a fail-safe for a disconnected/stalled encoder.
    constexpr float MOVE_TIMEOUT_FACTOR = 2.0f;
    const float duration_s =
        (std::fabs(command.value) / speed_mps) * MOVE_TIMEOUT_FACTOR;
    const auto duration_ms = static_cast<uint32_t>(std::ceil(duration_s * 1000.0f));
    move_until_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(std::max<uint32_t>(1U, duration_ms));

    context_.beginMove(
        command.value,
        speed_mps,
        odometry_service_.getTravelledDistanceM(),
        heading_service_.getHeadingDeg());
}

void MotionLoopTask::startTurn(const model::MotionCommand& command)
{
    if (!isValidTurn(command)) {
        failSafeStop();
        return;
    }

    if (!heading_service_.isInitialized()) {
        failSafeStop();
        return;
    }

    left_speed_pid_.reset();
    right_speed_pid_.reset();
    heading_service_.resetController();
    turn_direction_ = (command.value >= 0.0f) ? 1.0f : -1.0f;

    const float target_heading_deg = service::HeadingService::normalizeAngleDeg(
        heading_service_.getHeadingDeg() + command.value);

    context_.beginTurn(command.value, target_heading_deg);
}

void MotionLoopTask::startWait(const model::MotionCommand& command)
{
    if (!isValidWait(command)) {
        failSafeStop();
        return;
    }

    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    motor_service_.stopMotor();
    left_speed_pid_.reset();
    right_speed_pid_.reset();
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    wait_until_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(
        static_cast<uint32_t>(command.value * 1000.0f));
    context_.beginWait(command.value);
}

void MotionLoopTask::handleSnapshot(const MotionControlSnapshot& snapshot, float dt_s)
{
    switch (snapshot.state) {
    case model::MotionState::IDLE:
    case model::MotionState::COMPLETED:
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        break;
    case model::MotionState::MOVING:
        handleMove(snapshot, dt_s);
        break;
    case model::MotionState::TURNING:
        handleTurn(snapshot, dt_s);
        break;
    case model::MotionState::WAITING:
        handleWait(snapshot);
        break;
    case model::MotionState::STOPPING:
        stopAndIdle();
        break;
    case model::MotionState::ERROR:
        failSafeStop();
        break;
    }
}

void MotionLoopTask::handleWait(const MotionControlSnapshot& snapshot)
{
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    if (static_cast<int32_t>(xTaskGetTickCount() - wait_until_tick_) >= 0) {
        context_.setCompleted();
    }
    (void)snapshot;
}

void MotionLoopTask::handleMove(const MotionControlSnapshot& snapshot, float dt_s)
{
    const float travelled_distance_m = std::fabs(
        odometry_service_.getTravelledDistanceM() - snapshot.start_distance_m);
    const float target_distance_m = std::fabs(snapshot.target_distance_m);
    if (travelled_distance_m + config::robot::MOVE_DISTANCE_TOLERANCE_M >=
        target_distance_m) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        ESP_LOGI(kTag, "encoder distance target reached: travelled=%.3f target=%.3f",
                 static_cast<double>(travelled_distance_m),
                 static_cast<double>(target_distance_m));
        context_.setCompleted();
        return;
    }

    if (static_cast<int32_t>(xTaskGetTickCount() - move_until_tick_) >= 0) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        ESP_LOGW(kTag, "move timeout before encoder target: travelled=%.3f target=%.3f",
                 static_cast<double>(travelled_distance_m),
                 static_cast<double>(target_distance_m));
        context_.setCompleted();
        return;
    }

    const float direction = (snapshot.target_distance_m >= 0.0f) ? 1.0f : -1.0f;
    const float base_speed_mps = direction * snapshot.target_speed_mps;
    // The left wheel remains the speed master. IMU supervision of the right
    // follower is applied later, after the master speed has been measured.
    left_target_mps_ = base_speed_mps;
    right_target_mps_ = base_speed_mps;
    (void)dt_s;
}

void MotionLoopTask::handleTurn(const MotionControlSnapshot& snapshot, float dt_s)
{
    const float angle_error_deg = service::HeadingService::shortestAngleErrorDeg(
        snapshot.target_heading_deg,
        heading_service_.getHeadingDeg());

    const float remaining_angle_deg = turn_direction_ * angle_error_deg;
    // Stop slightly before the exact target to account for drivetrain inertia.
    // Never reverse to correct overshoot during this turn test.
    if (remaining_angle_deg <= config::robot::TURN_ANGLE_TOLERANCE_DEG) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        context_.setCompleted();
        return;
    }

    // Slow down over the final part of the turn to avoid a large inertial
    // overshoot while retaining a minimum output that can move the drivetrain.
    (void)dt_s;
    const float turn_output = std::clamp(
        config::robot::OPEN_LOOP_TURN_OUTPUT *
            (remaining_angle_deg / config::robot::TURN_SLOWDOWN_ANGLE_DEG),
        config::robot::MIN_TURN_OUTPUT,
        config::robot::OPEN_LOOP_TURN_OUTPUT);
    const float left_output = turn_direction_ * turn_output;
    const float right_output = -left_output;
    left_target_mps_ = turn_direction_;
    right_target_mps_ = -turn_direction_;
    previous_left_output_ = left_output;
    previous_right_output_ = right_output;
    motor_service_.setMotorOutputs(left_output, right_output);
}

void MotionLoopTask::stopAndIdle()
{
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    motor_service_.stopMotor();
    left_speed_pid_.reset();
    right_speed_pid_.reset();
    filtered_left_speed_mps_ = 0.0f;
    filtered_right_speed_mps_ = 0.0f;
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    feedback_filter_initialized_ = false;
    straight_heading_correction_ = StraightHeadingCorrection::NONE;
    context_.setIdle();
}

void MotionLoopTask::failSafeStop()
{
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    motor_service_.stopMotor();
    left_speed_pid_.reset();
    right_speed_pid_.reset();
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    straight_heading_correction_ = StraightHeadingCorrection::NONE;
    context_.setError();
}

void MotionLoopTask::applySpeedControl(float dt_s)
{
    if (context_.snapshot().state == model::MotionState::TURNING) {
        return;
    }

    if (std::fabs(left_target_mps_) <= 0.001f && std::fabs(right_target_mps_) <= 0.001f) {
        left_speed_pid_.reset();
        right_speed_pid_.reset();
        previous_left_output_ = 0.0f;
        previous_right_output_ = 0.0f;
        motor_service_.stopMotor();
        return;
    }

    const float left_speed_mps = motor_service_.getLeftVelocityMps();
    const float right_speed_mps = motor_service_.getRightVelocityMps();
    if (!feedback_filter_initialized_) {
        filtered_left_speed_mps_ = left_speed_mps;
        filtered_right_speed_mps_ = right_speed_mps;
        feedback_filter_initialized_ = true;
    } else {
        constexpr float alpha = config::pid::SPEED_FEEDBACK_FILTER_ALPHA;
        filtered_left_speed_mps_ += alpha * (left_speed_mps - filtered_left_speed_mps_);
        filtered_right_speed_mps_ += alpha * (right_speed_mps - filtered_right_speed_mps_);
    }

    float left_pid_target_mps = left_target_mps_;
    float right_pid_target_mps = right_target_mps_;
    // Master/follower synchronization is only for straight-line motion.
    // During a turn, the two wheel targets must keep opposite signs.
    if (context_.snapshot().state == model::MotionState::MOVING) {
        if (config::pid::SPEED_MASTER == config::pid::SpeedMaster::LEFT) {
            right_pid_target_mps = filtered_left_speed_mps_;

            // The measured hardware convention is:
            //   negative drift -> speed up the right follower
            //   positive drift -> slow down the right follower
            // Stateless, tiered heading correction. Large errors receive a
            // strong trim, medium errors a gentle trim, and errors inside the
            // deadband receive no IMU trim. This is not a heading PID.
            const MotionControlSnapshot snapshot = context_.snapshot();
            const float heading_drift_deg =
                service::HeadingService::shortestAngleErrorDeg(
                    heading_service_.getHeadingDeg(),
                    snapshot.target_heading_deg);

            if constexpr (config::robot::ENABLE_STRAIGHT_IMU_CORRECTION) {
                StraightHeadingCorrection requested_correction =
                    StraightHeadingCorrection::NONE;
                if (heading_drift_deg <=
                    -config::robot::STRAIGHT_HEADING_TRIGGER_DEG) {
                    requested_correction =
                        StraightHeadingCorrection::SPEED_UP_RIGHT;
                } else if (heading_drift_deg >=
                           config::robot::STRAIGHT_HEADING_TRIGGER_DEG) {
                    requested_correction =
                        StraightHeadingCorrection::SLOW_DOWN_RIGHT;
                }

                if (requested_correction != straight_heading_correction_) {
                    // Remove integral left by the previous correction mode.
                    right_speed_pid_.reset();
                    straight_heading_correction_ = requested_correction;
                }

                const float right_wheel_trim_mps =
                    std::fabs(heading_drift_deg) >=
                            config::robot::STRAIGHT_HEADING_LARGE_ERROR_DEG
                        ? config::robot::STRAIGHT_RIGHT_WHEEL_TRIM_MPS
                        : config::robot::STRAIGHT_RIGHT_WHEEL_FINE_TRIM_MPS;
                if (straight_heading_correction_ ==
                    StraightHeadingCorrection::SPEED_UP_RIGHT) {
                    right_pid_target_mps += right_wheel_trim_mps;
                } else if (straight_heading_correction_ ==
                           StraightHeadingCorrection::SLOW_DOWN_RIGHT) {
                    right_pid_target_mps -= right_wheel_trim_mps;
                }

                right_pid_target_mps = std::clamp(
                    right_pid_target_mps,
                    0.0f,
                    config::robot::MAX_WHEEL_SPEED_MPS);
            }
        } else if (config::pid::SPEED_MASTER == config::pid::SpeedMaster::RIGHT) {
            left_pid_target_mps = filtered_right_speed_mps_;
        }
    }

    const float left_output = left_speed_pid_.update(
        left_pid_target_mps, filtered_left_speed_mps_, dt_s) +
        (config::pid::SPEED_FEED_FORWARD * left_pid_target_mps);
    const float right_output = right_speed_pid_.update(
        right_pid_target_mps, filtered_right_speed_mps_, dt_s) +
        (config::pid::SPEED_FEED_FORWARD * right_pid_target_mps);
    const float max_output_step = config::pid::SPEED_OUTPUT_SLEW_RATE * dt_s;
    const float left_slewed = std::clamp(
        left_output, previous_left_output_ - max_output_step,
        previous_left_output_ + max_output_step);
    const float right_slewed = std::clamp(
        right_output, previous_right_output_ - max_output_step,
        previous_right_output_ + max_output_step);
    const float clamped_left_output = std::clamp(
        left_slewed, config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    const float clamped_right_output = std::clamp(
        right_slewed, config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    previous_left_output_ = clamped_left_output;
    previous_right_output_ = clamped_right_output;
    motor_service_.setMotorOutputs(
        clamped_left_output, clamped_right_output);
}

void MotionLoopTask::publishTelemetry()
{
    if (telemetry_queue_ == nullptr ||
        (xTaskGetTickCount() - last_telemetry_tick_) < pdMS_TO_TICKS(500)) {
        return;
    }

    const MotionControlSnapshot snapshot = context_.snapshot();
    model::TelemetryMessage message{};
    message.timestamp_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    message.level = model::LogLevel::INFO;
    std::snprintf(message.tag, sizeof(message.tag), "%s", "motion");
    std::snprintf(
        message.message, sizeof(message.message),
        "s=%u h=%.1f th=%.1f y=%.1f v=%.2f/%.2f o=%.2f/%.2f d=%.2f",
        static_cast<unsigned>(snapshot.state),
        static_cast<double>(heading_service_.getHeadingDeg()),
        static_cast<double>(snapshot.target_heading_deg),
        static_cast<double>(heading_service_.getYawRateDps()),
        static_cast<double>(motor_service_.getLeftVelocityMps()),
        static_cast<double>(motor_service_.getRightVelocityMps()),
        static_cast<double>(previous_left_output_),
        static_cast<double>(previous_right_output_),
        static_cast<double>(odometry_service_.getTravelledDistanceM()));
    (void)xQueueSend(telemetry_queue_, &message, 0);
    last_telemetry_tick_ = xTaskGetTickCount();
}

void MotionLoopTask::publishImuDiagnostic(
    const MotionControlSnapshot& snapshot)
{
    constexpr uint32_t MQTT_PERIOD_MS = 100;
    constexpr char MQTT_TOPIC[] = "diagnostic/imu/heading";

    if (snapshot.state != model::MotionState::MOVING) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    if (last_imu_mqtt_tick_ != 0 &&
        (now - last_imu_mqtt_tick_) < pdMS_TO_TICKS(MQTT_PERIOD_MS)) {
        return;
    }

    const float initial_heading_deg = snapshot.target_heading_deg;
    const float current_heading_deg = heading_service_.getHeadingDeg();
    const float delta_heading_deg =
        service::HeadingService::shortestAngleErrorDeg(
            current_heading_deg, initial_heading_deg);
    const int initial_heading = static_cast<int>(std::lround(initial_heading_deg));
    const int current_heading = static_cast<int>(std::lround(current_heading_deg));
    const int delta_heading = static_cast<int>(std::lround(delta_heading_deg));

    char payload[160]{};
    const int length = std::snprintf(
        payload,
        sizeof(payload),
        "{\"imu_initial_deg\":%d,\"imu_current_deg\":%d,\"imu_delta_deg\":%d}",
        initial_heading,
        current_heading,
        delta_heading);
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(payload)) {
        return;
    }

    if (mqtt_publish_queue_.pushJson(MQTT_TOPIC, payload, 0)) {
        last_imu_mqtt_tick_ = now;
        ESP_LOGI(kTag, "IMU MQTT queued: %s", payload);
    }
}

bool MotionLoopTask::servicesReady() const
{
    return motor_service_.isInitialized();
}

bool MotionLoopTask::isValidMove(const model::MotionCommand& command) const
{
    return std::isfinite(command.value) && std::isfinite(command.speed) &&
           std::fabs(command.value) > 0.0f && command.speed > 0.0f;
}

bool MotionLoopTask::isValidTurn(const model::MotionCommand& command) const
{
    return std::isfinite(command.value) && std::fabs(command.value) > 0.0f && std::fabs(command.value) <= 360.0f;
}

bool MotionLoopTask::isValidWait(const model::MotionCommand& command) const
{
    return std::isfinite(command.value) && command.value > 0.0f && command.value <= 300.0f;
}

}  // namespace minicar::logic::motion_control
