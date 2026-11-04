#pragma once

#include <cstdint>

namespace minicar::config::network {

// Intentionally blank until the Wi-Fi network and MQTT broker are selected.
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";

constexpr char MQTT_BROKER_URI[] = "";
constexpr char MQTT_USERNAME[] = "";
constexpr char MQTT_PASSWORD[] = "";
constexpr char MQTT_CLIENT_ID[] = "";  // Empty lets ESP-MQTT choose its default ID.

constexpr uint32_t WIFI_RETRY_PERIOD_MS = 5000;
constexpr uint32_t MQTT_RETRY_PERIOD_MS = 5000;
constexpr uint32_t CONNECTION_TASK_WAKE_PERIOD_MS = 100;
constexpr uint32_t MQTT_PUBLISH_QUEUE_LENGTH = 8;
constexpr uint32_t WIFI_MQTT_TASK_STACK_WORDS = 4096;
constexpr int WIFI_MQTT_TASK_PRIORITY = 3;
constexpr int MQTT_PUBLISH_QOS = 1;

}  // namespace minicar::config::network
