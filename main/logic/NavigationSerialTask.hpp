#pragma once

#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "logic/communication/MqttPublishEventQueue.hpp"
#include "service/navigation/NavigationSerialService.hpp"

namespace minicar::logic {

class NavigationSerialTask {
public:
    explicit NavigationSerialTask(
        service::NavigationSerialService& navigation_service,
        MqttPublishEventQueue& mqtt_publish_queue);

    esp_err_t init();
    esp_err_t start();

private:
    static void receiveTaskEntry(void* context);
    static void frameTaskEntry(void* context);
    void receiveTask();
    void frameTask();
    void consumeByte(char byte,
                     char* frame,
                     std::size_t& frame_length,
                     bool& discarding);
    void publishNavigationSample(const model::NavigationSample& sample);

    service::NavigationSerialService& navigation_service_;
    MqttPublishEventQueue& mqtt_publish_queue_;
    StreamBufferHandle_t stream_buffer_{nullptr};
    std::uint32_t last_mqtt_publish_ms_{0};
    bool has_published_mqtt_{false};
    std::uint8_t mqtt_samples_queued_{0};
    bool mqtt_session_complete_{false};
};

}  // namespace minicar::logic
