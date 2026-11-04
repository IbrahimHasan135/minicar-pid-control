#include "logic/NavigationSerialTask.hpp"

#include <cstdio>

#include "config/FreeRTOSConfig.hpp"
#include "config/NavigationSerialConfig.hpp"
#include "esp_log.h"
#include "model/NavigationSample.hpp"

namespace minicar::logic {
namespace {

constexpr char kTag[] = "NavigationSerial";
constexpr std::size_t kReadChunkSize = 64;

void valueToText(const model::NavigationValue& value,
                 char* output,
                 std::size_t output_size)
{
    if (!value.valid) {
        std::snprintf(output, output_size, "null");
        return;
    }
    std::snprintf(output, output_size, "%.3f", value.value);
}

}  // namespace

NavigationSerialTask::NavigationSerialTask(
    service::NavigationSerialService& navigation_service)
    : navigation_service_(navigation_service)
{
}

esp_err_t NavigationSerialTask::init()
{
    stream_buffer_ = xStreamBufferCreate(
        config::navigation_serial::STREAM_BUFFER_SIZE,
        config::navigation_serial::STREAM_TRIGGER_LEVEL);
    return stream_buffer_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t NavigationSerialTask::start()
{
    if (stream_buffer_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t result = xTaskCreate(
        frameTaskEntry,
        "nav_frame_parser",
        config::rtos::SERIAL_TASK_STACK_WORDS,
        this,
        config::rtos::SERIAL_TASK_PRIORITY,
        nullptr);
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    result = xTaskCreate(
        receiveTaskEntry,
        "nav_uart_rx",
        config::rtos::SERIAL_TASK_STACK_WORDS,
        this,
        config::rtos::SERIAL_TASK_PRIORITY,
        nullptr);
    return result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void NavigationSerialTask::receiveTaskEntry(void* context)
{
    static_cast<NavigationSerialTask*>(context)->receiveTask();
}

void NavigationSerialTask::frameTaskEntry(void* context)
{
    static_cast<NavigationSerialTask*>(context)->frameTask();
}

void NavigationSerialTask::receiveTask()
{
    std::uint8_t bytes[kReadChunkSize]{};
    while (true) {
        const int count = navigation_service_.readBytes(
            bytes,
            sizeof(bytes),
            config::navigation_serial::UART_READ_TIMEOUT_MS);
        if (count <= 0) {
            continue;
        }

        const std::size_t sent = xStreamBufferSend(
            stream_buffer_, bytes, static_cast<std::size_t>(count), 0);
        if (sent != static_cast<std::size_t>(count)) {
            ESP_LOGW(kTag, "RX stream buffer full; dropped %u bytes",
                     static_cast<unsigned>(count - sent));
        }
    }
}

void NavigationSerialTask::frameTask()
{
    char frame[config::navigation_serial::MAX_FRAME_LENGTH + 1]{};
    std::size_t frame_length = 0;
    bool discarding = false;
    std::uint8_t bytes[kReadChunkSize]{};

    while (true) {
        const std::size_t count = xStreamBufferReceive(
            stream_buffer_, bytes, sizeof(bytes), portMAX_DELAY);
        for (std::size_t i = 0; i < count; ++i) {
            consumeByte(static_cast<char>(bytes[i]),
                        frame,
                        frame_length,
                        discarding);
        }
    }
}

void NavigationSerialTask::consumeByte(char byte,
                                       char* frame,
                                       std::size_t& frame_length,
                                       bool& discarding)
{
    if (byte == '\r') {
        return;
    }

    if (byte != '\n') {
        if (discarding) {
            return;
        }
        if (frame_length >= config::navigation_serial::MAX_FRAME_LENGTH) {
            frame_length = 0;
            discarding = true;
            ESP_LOGW(kTag, "oversized frame discarded");
            return;
        }
        frame[frame_length++] = byte;
        return;
    }

    if (discarding) {
        discarding = false;
        frame_length = 0;
        return;
    }
    if (frame_length == 0) {
        return;
    }

    frame[frame_length] = '\0';
    model::NavigationSample sample{};
    if (navigation_service_.parseJsonLine(frame, sample)) {
        char values[4][24]{};
        for (std::size_t i = 0; i < 4; ++i) {
            valueToText(sample.values[i], values[i], sizeof(values[i]));
        }
        ESP_LOGI(kTag,
                 "valid sample: value1=%s value2=%s value3=%s value4=%s",
                 values[0], values[1], values[2], values[3]);
    } else {
        ESP_LOGW(kTag, "invalid navigation JSON rejected");
    }
    frame_length = 0;
}

}  // namespace minicar::logic
