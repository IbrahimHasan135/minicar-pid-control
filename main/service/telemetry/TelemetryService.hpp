#pragma once

#include "driver/telemetry/ConsoleDriver.hpp"
#include "esp_err.h"
#include "model/TelemetryMessage.hpp"
#include "service/base/Service.hpp"

namespace minicar::service {

class TelemetryService : public Service {
public:
    explicit TelemetryService(driver::ConsoleDriver& driver);

    esp_err_t init() override;
    void reset() override;

    esp_err_t publish(const model::TelemetryMessage& message);

private:
    const char* levelToText(model::LogLevel level) const;

    driver::ConsoleDriver& driver_;
    bool initialized_{false};
};

}  // namespace minicar::service
