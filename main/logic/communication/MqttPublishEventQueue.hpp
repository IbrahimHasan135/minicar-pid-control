#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "model/MqttPublishEvent.hpp"
#include "esp_err.h"

namespace minicar::logic {

// Logic-owned FreeRTOS queue shared by message producers and WifiMqttTask.
class MqttPublishEventQueue {
public:
    explicit MqttPublishEventQueue(QueueHandle_t queue = nullptr);

    esp_err_t create(uint32_t length);
    bool pushJson(const char* topic, const char* json_payload, TickType_t wait_ticks = 0);
    bool pull(model::MqttPublishEvent& event, TickType_t wait_ticks = 0) const;
    bool isReady() const;

private:
    QueueHandle_t queue_{nullptr};
};

}  // namespace minicar::logic
