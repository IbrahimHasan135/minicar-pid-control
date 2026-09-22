#include "driver/telemetry/ConsoleDriver.hpp"

#include <cstdio>

namespace minicar::driver {

esp_err_t ConsoleDriver::init()
{
    // TODO(Aranda): replace console stub if telemetry needs a dedicated UART.
    return ESP_OK;
}

esp_err_t ConsoleDriver::writeLine(const char* line, size_t length)
{
    if (line == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    std::printf("%.*s\n", static_cast<int>(length), line);
    return ESP_OK;
}

}  // namespace minicar::driver
