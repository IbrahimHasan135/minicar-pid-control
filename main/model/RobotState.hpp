#pragma once

#include "model/MotionState.hpp"
#include "model/Pose2D.hpp"

namespace minicar::model {

struct RobotState {
    float left_speed_mps{0.0f};
    float right_speed_mps{0.0f};
    float heading_deg{0.0f};
    float yaw_rate_dps{0.0f};
    float travelled_distance_m{0.0f};
    Pose2D pose{};
    MotionState motion_state{MotionState::IDLE};
};

}  // namespace minicar::model
