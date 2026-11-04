#pragma once

#include "driver/communication/MqttDriver.hpp"
#include "esp_err.h"
#include "model/MqttPublishEvent.hpp"

namespace minicar::service {

class MqttCommunicationService {
public:
    MqttCommunicationService(driver::MqttDriver& driver, const char* broker_uri,
                             const char* username, const char* password,
                             const char* client_id, int publish_qos);

    bool isConfigured() const;
    esp_err_t init();
    esp_err_t start();
    esp_err_t reconnect();
    esp_err_t disconnect();
    esp_err_t publish(const model::MqttPublishEvent& event);
    bool isConnected() const;
    bool isConnecting() const;

private:
    driver::MqttDriver& driver_;
    const char* broker_uri_;
    const char* username_;
    const char* password_;
    const char* client_id_;
    int publish_qos_;
};

}  // namespace minicar::service
