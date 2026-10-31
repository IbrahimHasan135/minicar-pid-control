#include "logic/motion_control/MotionLoopTask.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config/FreeRTOSConfig.hpp"
#include "config/PIDConfig.hpp"
#include "config/RobotConfig.hpp"

namespace minicar::logic::motion_control {

MotionLoopTask::MotionLoopTask(
    MotionControlContext& context,
    service::MotorControlService& motor_service,
    service::HeadingService& heading_service,
    service::OdometryService& odometry_service,
    QueueHandle_t telemetry_queue)
    : context_(context),
      motor_service_(motor_service),
      heading_service_(heading_service),
      odometry_service_(odometry_service),
      telemetry_queue_(telemetry_queue),
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

    while (true) {
        step(config::rtos::CONTROL_PERIOD_S);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(config::rtos::CONTROL_PERIOD_MS));
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
    heading_service_.resetController();

    const float speed_mps = std::clamp(
        std::fabs(command.speed),
        0.0f,
        config::robot::MAX_LINEAR_SPEED_MPS);

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

    const float target_heading_deg = service::HeadingService::normalizeAngleDeg(
        heading_service_.getHeadingDeg() + command.value);

    context_.beginTurn(command.value, target_heading_deg);
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
    case model::MotionState::STOPPING:
        stopAndIdle();
        break;
    case model::MotionState::ERROR:
        failSafeStop();
        break;
    }
}

void MotionLoopTask::handleMove(const MotionControlSnapshot& snapshot, float dt_s)
{
    const float travelled_m = odometry_service_.getTravelledDistanceM() - snapshot.start_distance_m;
    const float distance_error_m = snapshot.target_distance_m - travelled_m;
    const float abs_distance_error_m = std::fabs(distance_error_m);

    if (abs_distance_error_m <= config::robot::MOVE_DISTANCE_TOLERANCE_M &&
        motor_service_.getAverageSpeedMps() <= config::robot::MOVE_COMPLETE_SPEED_TOLERANCE_MPS) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        context_.setCompleted();
        return;
    }

    const float direction = (distance_error_m >= 0.0f) ? 1.0f : -1.0f;
    const float base_speed_mps = direction * snapshot.target_speed_mps;
    const float heading_correction_mps = heading_service_.calculateHeadingCorrection(
        snapshot.target_heading_deg,
        dt_s);

    left_target_mps_ = base_speed_mps - heading_correction_mps;
    right_target_mps_ = base_speed_mps + heading_correction_mps;
}

void MotionLoopTask::handleTurn(const MotionControlSnapshot& snapshot, float dt_s)
{
    const float angle_error_deg = service::HeadingService::shortestAngleErrorDeg(
        snapshot.target_heading_deg,
        heading_service_.getHeadingDeg());

    if (std::fabs(angle_error_deg) <= config::robot::TURN_ANGLE_TOLERANCE_DEG &&
        std::fabs(heading_service_.getYawRateDps()) <= config::robot::TURN_YAW_RATE_TOLERANCE_DPS) {
        left_target_mps_ = 0.0f;
        right_target_mps_ = 0.0f;
        motor_service_.stopMotor();
        context_.setCompleted();
        return;
    }

    const float turn_wheel_speed_mps = heading_service_.calculateHeadingCorrection(
        snapshot.target_heading_deg,
        dt_s);

    left_target_mps_ = turn_wheel_speed_mps;
    right_target_mps_ = -turn_wheel_speed_mps;
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
    context_.setError();
}

void MotionLoopTask::applySpeedControl(float dt_s)
{
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
    if (config::pid::SPEED_MASTER == config::pid::SpeedMaster::LEFT) {
        right_pid_target_mps = filtered_left_speed_mps_;
    } else if (config::pid::SPEED_MASTER == config::pid::SpeedMaster::RIGHT) {
        left_pid_target_mps = filtered_right_speed_mps_;
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
        "state=%u target=%.3f left=%.3f right=%.3f out=%.3f/%.3f dist=%.3f",
        static_cast<unsigned>(snapshot.state),
        static_cast<double>(snapshot.target_speed_mps),
        static_cast<double>(motor_service_.getLeftVelocityMps()),
        static_cast<double>(motor_service_.getRightVelocityMps()),
        static_cast<double>(previous_left_output_),
        static_cast<double>(previous_right_output_),
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

}  // namespace minicar::logic::motion_control
