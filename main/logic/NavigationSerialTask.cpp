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
    service::NavigationSerialService& navigation_service,
    MqttPublishEventQueue& mqtt_publish_queue)
    : navigation_service_(navigation_service),
      mqtt_publish_queue_(mqtt_publish_queue)
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
    TickType_t last_diagnostic_tick = xTaskGetTickCount();
    std::uint32_t total_bytes = 0;
    ESP_LOGI(kTag, "RX task started; waiting for newline-delimited JSON");
    while (true) {
        const int count = navigation_service_.readBytes(
            bytes,
            sizeof(bytes),
            config::navigation_serial::UART_READ_TIMEOUT_MS);
        if (count <= 0) {
            const TickType_t now = xTaskGetTickCount();
            if ((now - last_diagnostic_tick) >= pdMS_TO_TICKS(2000)) {
                ESP_LOGI(kTag,
                         "waiting: RX GPIO32 level=%d total_bytes=%lu",
                         navigation_service_.rxLevel(),
                         static_cast<unsigned long>(total_bytes));
                last_diagnostic_tick = now;
            }
            continue;
        }

        total_bytes += static_cast<std::uint32_t>(count);
        ESP_LOGI(kTag,
                 "received %d UART bytes (total=%lu)",
                 count,
                 static_cast<unsigned long>(total_bytes));

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
    ESP_LOGI(kTag, "raw frame: %s", frame);
    model::NavigationSample sample{};
    if (navigation_service_.parseJsonLine(frame, sample)) {
        if (sample.type == model::NavigationSampleType::RGBC) {
            ESP_LOGI(kTag, "valid RGBC serial sample");
        } else {
            ESP_LOGI(kTag, "valid RF serial sample: routers=%u",
                     static_cast<unsigned>(sample.router_count));
        }
        publishNavigationSample(sample);
    } else {
        ESP_LOGW(kTag, "invalid navigation JSON rejected");
    }
    frame_length = 0;
}

void NavigationSerialTask::publishNavigationSample(
    const model::NavigationSample& sample)
{
    if (mqtt_session_complete_) {
        return;
    }

    const std::uint32_t elapsed_ms =
        sample.received_at_ms - last_mqtt_publish_ms_;
    if (has_published_mqtt_ &&
        elapsed_ms < config::navigation_serial::MQTT_PUBLISH_PERIOD_MS) {
        return;
    }

    char payload[384]{};
    int payload_length = 0;
    const bool sending_end_marker =
        mqtt_samples_queued_ >=
        config::navigation_serial::MQTT_NAVIGATION_SAMPLE_LIMIT;
    if (sending_end_marker) {
        payload_length = std::snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"session\",\"status\":\"complete\",\"samples_sent\":%u}",
            static_cast<unsigned>(mqtt_samples_queued_));
    } else if (sample.type == model::NavigationSampleType::RGBC) {
        char values[4][24]{};
        for (std::size_t i = 0; i < 4; ++i) {
            valueToText(sample.values[i], values[i], sizeof(values[i]));
        }
        payload_length = std::snprintf(
            payload,
            sizeof(payload),
            "{\"type\":\"rgbc\",\"R\":%s,\"G\":%s,\"B\":%s,\"C\":%s}",
            values[0], values[1], values[2], values[3]);
    } else {
        payload_length = std::snprintf(
            payload, sizeof(payload), "{\"type\":\"rf\",\"routers\":[");
        for (std::uint8_t i = 0;
             i < sample.router_count && payload_length > 0;
             ++i) {
            const int appended = std::snprintf(
                payload + payload_length,
                sizeof(payload) - static_cast<std::size_t>(payload_length),
                "%s{\"short\":%u,\"rssi\":%d}",
                i == 0 ? "" : ",",
                static_cast<unsigned>(sample.routers[i].short_address),
                static_cast<int>(sample.routers[i].rssi));
            if (appended <= 0 ||
                appended >= static_cast<int>(sizeof(payload) - payload_length)) {
                payload_length = -1;
                break;
            }
            payload_length += appended;
        }
        if (payload_length > 0) {
            const int suffix = std::snprintf(
                payload + payload_length,
                sizeof(payload) - static_cast<std::size_t>(payload_length),
                "]}");
            if (suffix <= 0 ||
                suffix >= static_cast<int>(sizeof(payload) - payload_length)) {
                payload_length = -1;
            } else {
                payload_length += suffix;
            }
        }
    }
    if (payload_length <= 0 ||
        static_cast<std::size_t>(payload_length) >= sizeof(payload)) {
        ESP_LOGE(kTag, "navigation MQTT JSON formatting failed");
        return;
    }

    const bool queued = mqtt_publish_queue_.pushJson(
        config::navigation_serial::MQTT_NAVIGATION_TOPIC, payload, 0);
    if (queued) {
        last_mqtt_publish_ms_ = sample.received_at_ms;
        has_published_mqtt_ = true;
        if (sending_end_marker) {
            mqtt_session_complete_ = true;
            ESP_LOGI(kTag, "navigation MQTT session complete: payload=%s", payload);
        } else {
            ++mqtt_samples_queued_;
            ESP_LOGI(kTag, "navigation MQTT queued %u/%u: topic=%s payload=%s",
                     static_cast<unsigned>(mqtt_samples_queued_),
                     static_cast<unsigned>(
                         config::navigation_serial::MQTT_NAVIGATION_SAMPLE_LIMIT),
                     config::navigation_serial::MQTT_NAVIGATION_TOPIC,
                     payload);
        }
    } else {
        ESP_LOGW(kTag, "navigation MQTT queue full or unavailable");
    }
}

}  // namespace minicar::logic
