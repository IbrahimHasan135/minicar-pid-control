#include "service/communication/MqttCommunicationService.hpp"

namespace minicar::service {

MqttCommunicationService::MqttCommunicationService(
    driver::MqttDriver& driver, const char* broker_uri, const char* username,
    const char* password, const char* client_id, int publish_qos)
    : driver_(driver), broker_uri_(broker_uri), username_(username), password_(password),
      client_id_(client_id), publish_qos_(publish_qos)
{
}

bool MqttCommunicationService::isConfigured() const
{
    return broker_uri_ != nullptr && broker_uri_[0] != '\0';
}

esp_err_t MqttCommunicationService::init()
{
    if (!isConfigured()) return ESP_ERR_INVALID_STATE;
    return driver_.init(broker_uri_, username_, password_, client_id_);
}

esp_err_t MqttCommunicationService::start() { return driver_.start(); }
esp_err_t MqttCommunicationService::reconnect() { return driver_.reconnect(); }
esp_err_t MqttCommunicationService::disconnect() { return driver_.disconnect(); }
bool MqttCommunicationService::isConnected() const { return driver_.isConnected(); }
bool MqttCommunicationService::isConnecting() const { return driver_.isConnecting(); }

esp_err_t MqttCommunicationService::publish(const model::MqttPublishEvent& event)
{
    size_t topic_length = 0;
    while (topic_length < sizeof(event.topic) && event.topic[topic_length] != '\0') ++topic_length;
    size_t payload_length = 0;
    while (payload_length < sizeof(event.json_payload) && event.json_payload[payload_length] != '\0') ++payload_length;
    if (topic_length == 0 || topic_length >= sizeof(event.topic) ||
        payload_length == 0 || payload_length >= sizeof(event.json_payload)) return ESP_ERR_INVALID_ARG;
    return driver_.enqueueJson(event.topic, event.json_payload, payload_length, publish_qos_);
}

}  // namespace minicar::service
