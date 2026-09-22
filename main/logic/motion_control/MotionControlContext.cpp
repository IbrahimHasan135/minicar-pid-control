#include "logic/motion_control/MotionControlContext.hpp"

namespace minicar::logic::motion_control {

bool MotionControlContext::submitCommand(const model::MotionCommand& command)
{
    taskENTER_CRITICAL(&spinlock_);

    if (command.type == model::MotionType::STOP) {
        stop_requested_ = true;
        has_pending_command_ = false;
        taskEXIT_CRITICAL(&spinlock_);
        return true;
    }

    const bool can_accept = !has_pending_command_ && !stateIsBusyLocked();
    if (can_accept) {
        pending_command_ = command;
        has_pending_command_ = true;
    }

    taskEXIT_CRITICAL(&spinlock_);
    return can_accept;
}

void MotionControlContext::requestStop()
{
    taskENTER_CRITICAL(&spinlock_);
    stop_requested_ = true;
    has_pending_command_ = false;
    taskEXIT_CRITICAL(&spinlock_);
}

bool MotionControlContext::takePendingCommand(model::MotionCommand& command)
{
    taskENTER_CRITICAL(&spinlock_);

    const bool has_command = has_pending_command_;
    if (has_command) {
        command = pending_command_;
        has_pending_command_ = false;
    }

    taskEXIT_CRITICAL(&spinlock_);
    return has_command;
}

bool MotionControlContext::isBusy() const
{
    taskENTER_CRITICAL(&spinlock_);
    const bool busy = stateIsBusyLocked();
    taskEXIT_CRITICAL(&spinlock_);
    return busy;
}

bool MotionControlContext::hasStopRequest() const
{
    taskENTER_CRITICAL(&spinlock_);
    const bool requested = stop_requested_;
    taskEXIT_CRITICAL(&spinlock_);
    return requested;
}

void MotionControlContext::clearStopRequest()
{
    taskENTER_CRITICAL(&spinlock_);
    stop_requested_ = false;
    taskEXIT_CRITICAL(&spinlock_);
}

void MotionControlContext::beginMove(float distance_m, float speed_mps, float start_distance_m, float target_heading_deg)
{
    taskENTER_CRITICAL(&spinlock_);
    snapshot_.state = model::MotionState::MOVING;
    snapshot_.active_command = model::MotionCommand{model::MotionType::MOVE_DISTANCE, distance_m, speed_mps};
    snapshot_.target_distance_m = distance_m;
    snapshot_.target_speed_mps = speed_mps;
    snapshot_.start_distance_m = start_distance_m;
    snapshot_.target_heading_deg = target_heading_deg;
    taskEXIT_CRITICAL(&spinlock_);
}

void MotionControlContext::beginTurn(float angle_deg, float target_heading_deg)
{
    taskENTER_CRITICAL(&spinlock_);
    snapshot_.state = model::MotionState::TURNING;
    snapshot_.active_command = model::MotionCommand{model::MotionType::TURN_ANGLE, angle_deg, 0.0f};
    snapshot_.target_distance_m = 0.0f;
    snapshot_.target_speed_mps = 0.0f;
    snapshot_.start_distance_m = 0.0f;
    snapshot_.target_heading_deg = target_heading_deg;
    taskEXIT_CRITICAL(&spinlock_);
}

void MotionControlContext::setCompleted()
{
    taskENTER_CRITICAL(&spinlock_);
    snapshot_.state = model::MotionState::COMPLETED;
    taskEXIT_CRITICAL(&spinlock_);
}

void MotionControlContext::setIdle()
{
    taskENTER_CRITICAL(&spinlock_);
    snapshot_ = MotionControlSnapshot{};
    snapshot_.state = model::MotionState::IDLE;
    taskEXIT_CRITICAL(&spinlock_);
}

void MotionControlContext::setError()
{
    taskENTER_CRITICAL(&spinlock_);
    snapshot_.state = model::MotionState::ERROR;
    has_pending_command_ = false;
    taskEXIT_CRITICAL(&spinlock_);
}

MotionControlSnapshot MotionControlContext::snapshot() const
{
    taskENTER_CRITICAL(&spinlock_);
    const MotionControlSnapshot copy = snapshot_;
    taskEXIT_CRITICAL(&spinlock_);
    return copy;
}

bool MotionControlContext::stateIsBusyLocked() const
{
    return snapshot_.state == model::MotionState::MOVING ||
           snapshot_.state == model::MotionState::TURNING ||
           snapshot_.state == model::MotionState::STOPPING;
}

}  // namespace minicar::logic::motion_control
