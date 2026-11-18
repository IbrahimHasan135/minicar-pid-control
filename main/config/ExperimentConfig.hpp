#pragma once

#include <cstdint>

namespace minicar::config::experiment {

enum class SpeedProfile : std::uint8_t {
    V1_SLOW = 1,
    V2_MEDIUM = 2,
    V3_FAST = 3,
};

// Select one profile per track run. Rebuild/flash after changing this value.
constexpr SpeedProfile ACTIVE_SPEED_PROFILE = SpeedProfile::V1_SLOW;

constexpr float V1_SPEED_MPS = 0.20f;
constexpr float V2_SPEED_MPS = 0.40f;
constexpr float V3_SPEED_MPS = 0.60f;
constexpr float RUN_DISTANCE_M = 3.0f;
constexpr float RGBC_WARMUP_SECONDS = 1.0f;
constexpr float RGBC_SAMPLE_SPACING_M = 0.20f;
constexpr std::uint32_t RGBC_MAX_SAMPLE_AGE_MS = 500;

constexpr float activeSpeedMps()
{
    switch (ACTIVE_SPEED_PROFILE) {
    case SpeedProfile::V1_SLOW:
        return V1_SPEED_MPS;
    case SpeedProfile::V2_MEDIUM:
        return V2_SPEED_MPS;
    case SpeedProfile::V3_FAST:
        return V3_SPEED_MPS;
    }
    return V1_SPEED_MPS;
}

constexpr const char* activeProfileName()
{
    switch (ACTIVE_SPEED_PROFILE) {
    case SpeedProfile::V1_SLOW:
        return "V1";
    case SpeedProfile::V2_MEDIUM:
        return "V2";
    case SpeedProfile::V3_FAST:
        return "V3";
    }
    return "V1";
}

}  // namespace minicar::config::experiment
