#pragma once

#include <cstddef>

namespace minicar::model {

// Fixed-size, trivially copyable queue item. Producers fill topic and json_payload,
// then push the event; they never access Wi-Fi, MQTT, or a driver directly.
struct MqttPublishEvent {
    static constexpr size_t TOPIC_CAPACITY = 96;
    static constexpr size_t JSON_CAPACITY = 512;

    char topic[TOPIC_CAPACITY]{};
    char json_payload[JSON_CAPACITY]{};
};

}  // namespace minicar::model
