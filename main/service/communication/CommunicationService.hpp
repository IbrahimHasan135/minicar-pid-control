#pragma once

#include <cstdint>

#include "driver/communication/SerialDriver.hpp"
#include "esp_err.h"
#include "model/MotionCommand.hpp"
#include "service/base/Service.hpp"

namespace minicar::service {

class CommunicationService : public Service {
public:
    explicit CommunicationService(driver::SerialDriver& driver);

    esp_err_t init() override;
    void reset() override;

    bool readCommand(model::MotionCommand& command, uint32_t timeout_ms);

private:
    bool parseCommandLine(const char* line, model::MotionCommand& command) const;

    driver::SerialDriver& driver_;
    bool initialized_{false};
};

}  // namespace minicar::service
