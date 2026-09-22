#pragma once

#include <cstdint>

namespace minicar::model {

enum class LogLevel : uint8_t {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
};

struct TelemetryMessage {
    uint32_t timestamp_ms{0};
    LogLevel level{LogLevel::INFO};
    char tag[16]{};
    char message[96]{};
};

}  // namespace minicar::model
