#include "logic/communication/MqttPublishEventQueue.hpp"

#include <cstring>

namespace minicar::logic {

MqttPublishEventQueue::MqttPublishEventQueue(QueueHandle_t queue) : queue_(queue) {}

esp_err_t MqttPublishEventQueue::create(uint32_t length)
{
    if (queue_ != nullptr || length == 0) return ESP_ERR_INVALID_STATE;
    queue_ = xQueueCreate(length, sizeof(model::MqttPublishEvent));
    return queue_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM;
}

bool MqttPublishEventQueue::pushJson(const char* topic, const char* json_payload, TickType_t wait_ticks)
{
    if (queue_ == nullptr || topic == nullptr || json_payload == nullptr ||
        topic[0] == '\0' || json_payload[0] == '\0') return false;

    const size_t topic_length = std::strlen(topic);
    const size_t payload_length = std::strlen(json_payload);
    if (topic_length >= model::MqttPublishEvent::TOPIC_CAPACITY ||
        payload_length >= model::MqttPublishEvent::JSON_CAPACITY) return false;

    model::MqttPublishEvent event{};
    std::memcpy(event.topic, topic, topic_length);
    std::memcpy(event.json_payload, json_payload, payload_length);
    return xQueueSend(queue_, &event, wait_ticks) == pdTRUE;
}

bool MqttPublishEventQueue::pull(model::MqttPublishEvent& event, TickType_t wait_ticks) const
{
    return queue_ != nullptr && xQueueReceive(queue_, &event, wait_ticks) == pdTRUE;
}

bool MqttPublishEventQueue::isReady() const { return queue_ != nullptr; }

}  // namespace minicar::logic
