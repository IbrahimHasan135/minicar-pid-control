#pragma once

#include <cstdint>

namespace minicar::model {

enum class MotionType : uint8_t {
    MOVE_DISTANCE,
    TURN_ANGLE,
    STOP,
};

struct MotionCommand {
    MotionType type{MotionType::STOP};
    float value{0.0f};
    float speed{0.0f};
};

}  // namespace minicar::model
