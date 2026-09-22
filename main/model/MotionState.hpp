#pragma once

#include <cstdint>

namespace minicar::model {

enum class MotionState : uint8_t {
    IDLE,
    MOVING,
    TURNING,
    STOPPING,
    COMPLETED,
    ERROR,
};

}  // namespace minicar::model
