#include "driver/navigation/NavigationSerialDriver.hpp"

#include "config/NavigationSerialConfig.hpp"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

namespace minicar::driver {

esp_err_t NavigationSerialDriver::init()
{
    using namespace config::navigation_serial;

    uart_config_t uart_config{};
    uart_config.baud_rate = BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_param_config(UART_PORT, &uart_config);
    if (result == ESP_OK) {
        result = uart_set_pin(
            UART_PORT,
            UART_PIN_NO_CHANGE,
            RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE);
    }
    if (result == ESP_OK) {
        result = uart_driver_install(
            UART_PORT,
            UART_RX_BUFFER_SIZE,
            0,
            0,
            nullptr,
            0);
    }

    initialized_ = (result == ESP_OK);
    return result;
}

int NavigationSerialDriver::readBytes(std::uint8_t* buffer,
                                      std::size_t buffer_size,
                                      std::uint32_t timeout_ms)
{
    if (!initialized_ || buffer == nullptr || buffer_size == 0) {
        return -1;
    }

    return uart_read_bytes(
        config::navigation_serial::UART_PORT,
        buffer,
        buffer_size,
        pdMS_TO_TICKS(timeout_ms));
}

}  // namespace minicar::driver
