#pragma once

#include <atomic>
#include <cstddef>

#include "esp_err.h"
#include "mqtt_client.h"

namespace minicar::driver {

class MqttDriver {
public:
    esp_err_t init(const char* broker_uri, const char* username,
                   const char* password, const char* client_id);
    esp_err_t start();
    esp_err_t reconnect();
    esp_err_t disconnect();
    esp_err_t enqueueJson(const char* topic, const char* payload, size_t payload_length,
                          int qos, bool retain = false);
    bool isConnected() const;
    bool isConnecting() const;

private:
    static void eventHandler(void* handler_argument, esp_event_base_t event_base,
                             int32_t event_id, void* event_data);
    void handleEvent(int32_t event_id, esp_mqtt_event_handle_t event);

    esp_mqtt_client_handle_t client_{nullptr};
    std::atomic<bool> connected_{false};
    std::atomic<bool> connecting_{false};
    bool started_{false};
};

}  // namespace minicar::driver
