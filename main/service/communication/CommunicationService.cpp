#include "service/communication/CommunicationService.hpp"

#include <cstdlib>
#include <cstring>

namespace minicar::service {

CommunicationService::CommunicationService(driver::SerialDriver& driver)
    : driver_(driver)
{
}

esp_err_t CommunicationService::init()
{
    const esp_err_t result = driver_.init();
    initialized_ = (result == ESP_OK);
    return result;
}

void CommunicationService::reset()
{
}

bool CommunicationService::readCommand(model::MotionCommand& command, uint32_t timeout_ms)
{
    if (!initialized_) {
        return false;
    }

    char line[64]{};
    if (!driver_.readLine(line, sizeof(line), timeout_ms)) {
        return false;
    }

    return parseCommandLine(line, command);
}

bool CommunicationService::parseCommandLine(const char* line, model::MotionCommand& command) const
{
    if (line == nullptr) {
        return false;
    }

    if (std::strncmp(line, "STOP", 4) == 0) {
        command = model::MotionCommand{model::MotionType::STOP, 0.0f, 0.0f};
        return true;
    }

    if (std::strncmp(line, "MOVE", 4) == 0) {
        char* end_ptr = nullptr;
        const float distance_m = std::strtof(line + 4, &end_ptr);
        if (end_ptr == line + 4) {
            return false;
        }

        const float speed_mps = std::strtof(end_ptr, &end_ptr);
        if (speed_mps <= 0.0f) {
            return false;
        }

        command = model::MotionCommand{model::MotionType::MOVE_DISTANCE, distance_m, speed_mps};
        return true;
    }

    if (std::strncmp(line, "TURN", 4) == 0) {
        char* end_ptr = nullptr;
        const float angle_deg = std::strtof(line + 4, &end_ptr);
        if (end_ptr == line + 4) {
            return false;
        }

        command = model::MotionCommand{model::MotionType::TURN_ANGLE, angle_deg, 0.0f};
        return true;
    }

    return false;
}

}  // namespace minicar::service
