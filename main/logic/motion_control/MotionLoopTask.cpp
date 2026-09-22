#include "logic/motion_control/MotionLoopTask.hpp"

#include <algorithm>
#include <cmath>

#include "config/FreeRTOSConfig.hpp"
#include "config/RobotConfig.hpp"

namespace minicar::logic::motion_control {

MotionLoopTask::MotionLoopTask(
    MotionControlContext& context,
    service::MotorControlService& motor_service,
    service::HeadingService& heading_service,
    service::OdometryService& odometry_service)
    : context_(context),
      motor_service_(motor_service),
      heading_service_(heading_service),
      odometry_service_(odometry_service)
{
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
    motor_service_.applyControl(dt_s);
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

    motor_service_.resetControllers();
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

    motor_service_.resetControllers();
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
        motor_service_.setVelocityTargets(0.0f, 0.0f);
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
        motor_service_.stopMotor();
        context_.setCompleted();
        return;
    }

    const float direction = (distance_error_m >= 0.0f) ? 1.0f : -1.0f;
    const float base_speed_mps = direction * snapshot.target_speed_mps;
    const float heading_correction_mps = heading_service_.calculateHeadingCorrection(
        snapshot.target_heading_deg,
        dt_s);

    motor_service_.setVelocityTargets(
        base_speed_mps - heading_correction_mps,
        base_speed_mps + heading_correction_mps);
}

void MotionLoopTask::handleTurn(const MotionControlSnapshot& snapshot, float dt_s)
{
    const float angle_error_deg = service::HeadingService::shortestAngleErrorDeg(
        snapshot.target_heading_deg,
        heading_service_.getHeadingDeg());

    if (std::fabs(angle_error_deg) <= config::robot::TURN_ANGLE_TOLERANCE_DEG &&
        std::fabs(heading_service_.getYawRateDps()) <= config::robot::TURN_YAW_RATE_TOLERANCE_DPS) {
        motor_service_.stopMotor();
        context_.setCompleted();
        return;
    }

    const float turn_wheel_speed_mps = heading_service_.calculateHeadingCorrection(
        snapshot.target_heading_deg,
        dt_s);

    motor_service_.setVelocityTargets(turn_wheel_speed_mps, -turn_wheel_speed_mps);
}

void MotionLoopTask::stopAndIdle()
{
    motor_service_.stopMotor();
    context_.setIdle();
}

void MotionLoopTask::failSafeStop()
{
    motor_service_.stopMotor();
    context_.setError();
}

bool MotionLoopTask::servicesReady() const
{
    return motor_service_.isInitialized() && heading_service_.isInitialized();
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
