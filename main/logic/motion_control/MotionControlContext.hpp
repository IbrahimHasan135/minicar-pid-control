#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "model/MotionCommand.hpp"
#include "model/MotionState.hpp"

namespace minicar::logic::motion_control {

struct MotionControlSnapshot {
    model::MotionState state{model::MotionState::IDLE};
    model::MotionCommand active_command{};
    float target_distance_m{0.0f};
    float target_speed_mps{0.0f};
    float start_distance_m{0.0f};
    float target_heading_deg{0.0f};
};

class MotionControlContext {
public:
    bool submitCommand(const model::MotionCommand& command);
    void requestStop();

    bool takePendingCommand(model::MotionCommand& command);
    bool isBusy() const;
    bool hasStopRequest() const;
    void clearStopRequest();

    void beginMove(float distance_m, float speed_mps, float start_distance_m, float target_heading_deg);
    void beginTurn(float angle_deg, float target_heading_deg);
    void setCompleted();
    void setIdle();
    void setError();

    MotionControlSnapshot snapshot() const;

private:
    bool stateIsBusyLocked() const;

    mutable portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;
    MotionControlSnapshot snapshot_{};
    model::MotionCommand pending_command_{};
    bool has_pending_command_{false};
    bool stop_requested_{false};
};

}  // namespace minicar::logic::motion_control
