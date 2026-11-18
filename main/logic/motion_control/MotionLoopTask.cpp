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
    ramped_move_target_mps_ = 0.0f;
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    heading_error_deg_ = 0.0f;
    heading_speed_correction_mps_ = 0.0f;
    encoder_distance_error_m_ = 0.0f;
    encoder_sync_correction_mps_ = 0.0f;
    right_pid_target_mps_ = 0.0f;
    move_start_left_ticks_ = motor_service_.getLeftTicks();
    move_start_right_ticks_ = motor_service_.getRightTicks();
    feedback_filter_initialized_ = false;
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
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    heading_error_deg_ = 0.0f;
    turn_direction_ = (command.value >= 0.0f) ? 1.0f : -1.0f;
    turn_target_angle_deg_ = std::fabs(command.value);
    turn_raw_progress_deg_ = 0.0f;
    turn_progress_deg_ = 0.0f;
    turn_remaining_deg_ = turn_target_angle_deg_;
    turn_sync_output_ = 0.0f;
    turn_start_left_ticks_ = motor_service_.getLeftTicks();
    turn_start_right_ticks_ = motor_service_.getRightTicks();

    const float start_heading_deg = heading_service_.getHeadingDeg();
    // Positive turn commands are defined as a physical right turn. On this
    // drivetrain/IMU installation a right turn decreases magnetic heading.
    const float target_heading_deg = service::HeadingService::normalizeAngleDeg(
        start_heading_deg - command.value);
    turn_until_tick_ = xTaskGetTickCount() +
        pdMS_TO_TICKS(config::robot::TURN_TIMEOUT_MS);

    ESP_LOGI(
        kTag,
        "turn started: angle=%.1f start=%.2f target=%.2f timeout=%lu ms",
        static_cast<double>(command.value),
        static_cast<double>(start_heading_deg),
        static_cast<double>(target_heading_deg),
        static_cast<unsigned long>(config::robot::TURN_TIMEOUT_MS));

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
    heading_error_deg_ = angle_error_deg;

    constexpr float PI = 3.14159265358979323846f;
    constexpr float METERS_PER_TICK =
        (config::robot::WHEEL_DIAMETER_M * PI) /
        static_cast<float>(config::robot::ENCODER_TICKS_PER_REV);
    const int32_t left_turn_ticks = std::abs(
        motor_service_.getLeftTicks() - turn_start_left_ticks_);
    const int32_t right_turn_ticks = std::abs(
        motor_service_.getRightTicks() - turn_start_right_ticks_);
    const float left_turn_distance_m =
        static_cast<float>(left_turn_ticks) * METERS_PER_TICK;
    const float right_turn_distance_m =
        static_cast<float>(right_turn_ticks) * METERS_PER_TICK;
    const float average_wheel_arc_m =
        0.5f * (left_turn_distance_m + right_turn_distance_m);
    turn_raw_progress_deg_ = average_wheel_arc_m /
        (config::robot::WHEEL_BASE_M * 0.5f) * (180.0f / PI);
    turn_progress_deg_ = turn_raw_progress_deg_ /
        config::robot::TURN_ENCODER_ARC_MULTIPLIER;
    turn_remaining_deg_ = std::max(
        0.0f, turn_target_angle_deg_ - turn_progress_deg_);

    if (static_cast<int32_t>(xTaskGetTickCount() - turn_until_tick_) >= 0) {
        motor_service_.stopMotor();
        ESP_LOGE(
            kTag,
            "turn timeout: target=%.2f heading=%.2f error=%.2f",
            static_cast<double>(snapshot.target_heading_deg),
            static_cast<double>(heading_service_.getHeadingDeg()),
            static_cast<double>(angle_error_deg));
        failSafeStop();
        return;
    }

    // Encoder wheel arc is the primary completion condition. IMU heading stays
    // available as an independent diagnostic and cannot cause a full rotation.
    if (turn_remaining_deg_ <= config::robot::TURN_ANGLE_TOLERANCE_DEG) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        previous_left_output_ = 0.0f;
        previous_right_output_ = 0.0f;
        ESP_LOGI(
            kTag,
            "turn target reached: encoder_angle=%.2f target_angle=%.2f "
            "heading_target=%.2f heading=%.2f heading_error=%.2f yaw=%.2f",
            static_cast<double>(turn_progress_deg_),
            static_cast<double>(turn_target_angle_deg_),
            static_cast<double>(snapshot.target_heading_deg),
            static_cast<double>(heading_service_.getHeadingDeg()),
            static_cast<double>(angle_error_deg),
            static_cast<double>(heading_service_.getYawRateDps()));
        context_.setCompleted();
        return;
    }

    // Slow down over the final part of the turn to avoid a large inertial
    // overshoot while retaining a minimum output that can move the drivetrain.
    (void)dt_s;
    const float turn_output = std::clamp(
        config::robot::OPEN_LOOP_TURN_OUTPUT *
            (turn_remaining_deg_ / config::robot::TURN_SLOWDOWN_ANGLE_DEG),
        config::robot::MIN_TURN_OUTPUT,
        config::robot::OPEN_LOOP_TURN_OUTPUT);
    const float turn_tick_error =
        static_cast<float>(left_turn_ticks - right_turn_ticks);
    turn_sync_output_ = std::clamp(
        config::robot::TURN_SYNC_OUTPUT_PER_TICK * turn_tick_error,
        -config::robot::TURN_SYNC_MAX_OUTPUT,
        config::robot::TURN_SYNC_MAX_OUTPUT);
    const float left_output_magnitude = std::clamp(
        turn_output - turn_sync_output_,
        config::robot::TURN_MIN_WHEEL_OUTPUT,
        config::robot::TURN_MAX_WHEEL_OUTPUT);
    const float right_output_magnitude = std::clamp(
        turn_output + turn_sync_output_,
        config::robot::TURN_MIN_WHEEL_OUTPUT,
        config::robot::TURN_MAX_WHEEL_OUTPUT);
    const float left_output = turn_direction_ * left_output_magnitude;
    const float right_output = -turn_direction_ * right_output_magnitude;
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
    ramped_move_target_mps_ = 0.0f;
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    heading_error_deg_ = 0.0f;
    heading_speed_correction_mps_ = 0.0f;
    encoder_distance_error_m_ = 0.0f;
    encoder_sync_correction_mps_ = 0.0f;
    right_pid_target_mps_ = 0.0f;
    feedback_filter_initialized_ = false;
    context_.setIdle();
}

void MotionLoopTask::failSafeStop()
{
    left_target_mps_ = 0.0f;
    right_target_mps_ = 0.0f;
    motor_service_.stopMotor();
    left_speed_pid_.reset();
    right_speed_pid_.reset();
    ramped_move_target_mps_ = 0.0f;
    previous_left_output_ = 0.0f;
    previous_right_output_ = 0.0f;
    heading_error_deg_ = 0.0f;
    heading_speed_correction_mps_ = 0.0f;
    encoder_distance_error_m_ = 0.0f;
    encoder_sync_correction_mps_ = 0.0f;
    right_pid_target_mps_ = 0.0f;
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
        ramped_move_target_mps_ = 0.0f;
        previous_left_output_ = 0.0f;
        previous_right_output_ = 0.0f;
        right_pid_target_mps_ = 0.0f;
        heading_speed_correction_mps_ = 0.0f;
        encoder_sync_correction_mps_ = 0.0f;
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
    heading_error_deg_ = 0.0f;
    heading_speed_correction_mps_ = 0.0f;
    encoder_distance_error_m_ = 0.0f;
    encoder_sync_correction_mps_ = 0.0f;
    right_pid_target_mps_ = right_pid_target_mps;
    // Master/follower synchronization is only for straight-line motion.
    // During a turn, the two wheel targets must keep opposite signs.
    if (context_.snapshot().state == model::MotionState::MOVING) {
        const float max_target_step =
            config::pid::MOVE_TARGET_ACCELERATION_MPS2 * dt_s;
        ramped_move_target_mps_ += std::clamp(
            left_target_mps_ - ramped_move_target_mps_,
            -max_target_step,
            max_target_step);
        left_pid_target_mps = ramped_move_target_mps_;
        right_pid_target_mps = ramped_move_target_mps_;

        if (config::pid::SPEED_MASTER == config::pid::SpeedMaster::LEFT) {
            constexpr float METERS_PER_TICK =
                (config::robot::WHEEL_DIAMETER_M * 3.14159265358979323846f) /
                static_cast<float>(config::robot::ENCODER_TICKS_PER_REV);
            const int32_t left_delta_ticks =
                motor_service_.getLeftTicks() - move_start_left_ticks_;
            const int32_t right_delta_ticks =
                motor_service_.getRightTicks() - move_start_right_ticks_;
            const float travel_direction =
                left_target_mps_ >= 0.0f ? 1.0f : -1.0f;
            const float desired_right_ticks =
                static_cast<float>(left_delta_ticks) *
                (1.0f + config::pid::RIGHT_ENCODER_DISTANCE_LEAD_RATIO);
            encoder_distance_error_m_ = travel_direction *
                (desired_right_ticks - static_cast<float>(right_delta_ticks)) *
                METERS_PER_TICK;
            encoder_sync_correction_mps_ = travel_direction * std::clamp(
                config::pid::ENCODER_SYNC_KP_PER_S * encoder_distance_error_m_,
                -config::pid::ENCODER_SYNC_MAX_CORRECTION_MPS,
                config::pid::ENCODER_SYNC_MAX_CORRECTION_MPS);

            // Cascade control: heading adjusts the right wheel speed target;
            // the right speed PID is the only controller that drives PWM.
            const MotionControlSnapshot snapshot = context_.snapshot();
            heading_error_deg_ =
                service::HeadingService::shortestAngleErrorDeg(
                    snapshot.target_heading_deg,
                    heading_service_.getHeadingDeg());

            if constexpr (config::robot::ENABLE_STRAIGHT_IMU_CORRECTION) {
                const float absolute_heading_error =
                    std::fabs(heading_error_deg_);
                if (absolute_heading_error >
                        config::robot::STRAIGHT_HEADING_DEADBAND_DEG &&
                    absolute_heading_error <=
                        config::robot::STRAIGHT_HEADING_VALID_ERROR_DEG) {
                    const float damped_yaw_rate_dps = std::clamp(
                        heading_service_.getYawRateDps(),
                        -config::robot::STRAIGHT_HEADING_YAW_RATE_LIMIT_DPS,
                        config::robot::STRAIGHT_HEADING_YAW_RATE_LIMIT_DPS);
                    heading_speed_correction_mps_ = std::clamp(
                        config::robot::STRAIGHT_HEADING_KP_SPEED_PER_DEG *
                                heading_error_deg_ -
                            config::robot::STRAIGHT_HEADING_KD_SPEED_PER_DPS *
                                damped_yaw_rate_dps,
                        -config::robot::STRAIGHT_HEADING_MAX_SPEED_CORRECTION_MPS,
                        config::robot::STRAIGHT_HEADING_MAX_SPEED_CORRECTION_MPS);
                }
            }
            right_pid_target_mps = std::clamp(
                ramped_move_target_mps_ + encoder_sync_correction_mps_ +
                    heading_speed_correction_mps_,
                -config::robot::MAX_WHEEL_SPEED_MPS,
                config::robot::MAX_WHEEL_SPEED_MPS);
            right_pid_target_mps_ = right_pid_target_mps;
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
        "s=%u h=%.1f t=%.1f e=%.1f v=%.2f/%.2f rt=%.2f hc=%.3f o=%.2f/%.2f d=%.2f",
        static_cast<unsigned>(snapshot.state),
        static_cast<double>(heading_service_.getHeadingDeg()),
        static_cast<double>(snapshot.target_heading_deg),
        static_cast<double>(heading_error_deg_),
        static_cast<double>(motor_service_.getLeftVelocityMps()),
        static_cast<double>(motor_service_.getRightVelocityMps()),
        static_cast<double>(right_pid_target_mps_),
        static_cast<double>(heading_speed_correction_mps_),
        static_cast<double>(previous_left_output_),
        static_cast<double>(previous_right_output_),
        static_cast<double>(odometry_service_.getTravelledDistanceM()));
    ESP_LOGI(
        kTag,
        "CTRL heading=%.2f target=%.2f error=%.2f yaw=%.1f "
        "turn_raw=%.1f turn_progress=%.1f turn_remaining=%.1f turn_sync=%.3f "
        "speed_L/R=%.3f/%.3f move_target=%.3f right_target=%.3f heading_corr=%.3f "
        "encoder_error=%.3f encoder_corr=%.3f "
        "output_L/R=%.3f/%.3f ticks_L/R=%ld/%ld distance=%.3f",
        static_cast<double>(heading_service_.getHeadingDeg()),
        static_cast<double>(snapshot.target_heading_deg),
        static_cast<double>(heading_error_deg_),
        static_cast<double>(heading_service_.getYawRateDps()),
        static_cast<double>(turn_raw_progress_deg_),
        static_cast<double>(turn_progress_deg_),
        static_cast<double>(turn_remaining_deg_),
        static_cast<double>(turn_sync_output_),
        static_cast<double>(motor_service_.getLeftVelocityMps()),
        static_cast<double>(motor_service_.getRightVelocityMps()),
        static_cast<double>(ramped_move_target_mps_),
        static_cast<double>(right_pid_target_mps_),
        static_cast<double>(heading_speed_correction_mps_),
        static_cast<double>(encoder_distance_error_m_),
        static_cast<double>(encoder_sync_correction_mps_),
        static_cast<double>(previous_left_output_),
        static_cast<double>(previous_right_output_),
        static_cast<long>(motor_service_.getLeftTicks()),
        static_cast<long>(motor_service_.getRightTicks()),
        static_cast<double>(odometry_service_.getTravelledDistanceM()));
    (void)xQueueSend(telemetry_queue_, &message, 0);
    last_telemetry_tick_ = xTaskGetTickCount();
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
