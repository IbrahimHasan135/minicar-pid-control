#include "driver/communication/SerialDriver.hpp"

namespace minicar::driver {

esp_err_t SerialDriver::init()
{
    // TODO(Aranda): configure external command UART if this project uses one.
    return ESP_ERR_NOT_SUPPORTED;
}

bool SerialDriver::readLine(char* buffer, size_t buffer_size, uint32_t timeout_ms)
{
    (void)buffer;
    (void)buffer_size;
    (void)timeout_ms;
    // TODO(Aranda): read one complete command frame/line from UART.
    return false;
}

}  // namespace minicar::driver
