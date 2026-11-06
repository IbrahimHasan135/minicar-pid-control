#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"

namespace minicar::config::navigation_serial {

constexpr uart_port_t UART_PORT = UART_NUM_2;
constexpr gpio_num_t RX_GPIO = GPIO_NUM_32;
constexpr int BAUD_RATE = 115200;
constexpr std::size_t UART_RX_BUFFER_SIZE = 512;
constexpr std::size_t STREAM_BUFFER_SIZE = 512;
constexpr std::size_t STREAM_TRIGGER_LEVEL = 1;
constexpr std::size_t MAX_FRAME_LENGTH = 128;
constexpr std::uint32_t UART_READ_TIMEOUT_MS = 20;
constexpr std::uint32_t MQTT_PUBLISH_PERIOD_MS = 5000;
constexpr std::uint8_t MQTT_RGBC_SAMPLE_LIMIT = 5;
// Shared topic for the interchangeable RGBC and RF serial navigation nodes.
constexpr char MQTT_RGBC_TOPIC[] = "serial-communication/rgbc-rf";

}  // namespace minicar::config::navigation_serial
