#include "service/telemetry/TelemetryService.hpp"

#include <cstdio>

namespace minicar::service {

TelemetryService::TelemetryService(driver::ConsoleDriver& driver)
    : driver_(driver)
{
}

esp_err_t TelemetryService::init()
{
    const esp_err_t result = driver_.init();
    initialized_ = (result == ESP_OK);
    return result;
}

void TelemetryService::reset()
{
}

esp_err_t TelemetryService::publish(const model::TelemetryMessage& message)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    char line[160]{};
    const int length = std::snprintf(
        line,
        sizeof(line),
        "[%lu] %s %s: %s",
        static_cast<unsigned long>(message.timestamp_ms),
        levelToText(message.level),
        message.tag,
        message.message);

    if (length <= 0) {
        return ESP_FAIL;
    }

    const size_t bounded_length = static_cast<size_t>(length) < sizeof(line) ? static_cast<size_t>(length) : sizeof(line) - 1;
    return driver_.writeLine(line, bounded_length);
}

const char* TelemetryService::levelToText(model::LogLevel level) const
{
    switch (level) {
    case model::LogLevel::DEBUG:
        return "DEBUG";
    case model::LogLevel::INFO:
        return "INFO";
    case model::LogLevel::WARNING:
        return "WARN";
    case model::LogLevel::ERROR:
        return "ERROR";
    }

    return "INFO";
}

}  // namespace minicar::service
