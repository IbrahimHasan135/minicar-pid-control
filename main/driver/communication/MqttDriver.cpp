#include "driver/communication/MqttDriver.hpp"

#include "esp_log.h"

namespace minicar::driver {
namespace {
constexpr char TAG[] = "MqttDriver";
}

esp_err_t MqttDriver::init(const char* broker_uri, const char* username,
                           const char* password, const char* client_id)
{
    if (broker_uri == nullptr || broker_uri[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (client_ != nullptr) return ESP_ERR_INVALID_STATE;

    esp_mqtt_client_config_t config{};
    config.broker.address.uri = broker_uri;
    if (username != nullptr && username[0] != '\0') config.credentials.username = username;
    if (password != nullptr && password[0] != '\0') config.credentials.authentication.password = password;
    if (client_id != nullptr && client_id[0] != '\0') config.credentials.client_id = client_id;
    config.network.disable_auto_reconnect = true;  // WifiMqttTask owns retry/watchdog policy.
    config.session.keepalive = 30;

    client_ = esp_mqtt_client_init(&config);
    if (client_ == nullptr) return ESP_ERR_NO_MEM;
    const esp_err_t result = esp_mqtt_client_register_event(
        client_, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        &MqttDriver::eventHandler, this);
    if (result != ESP_OK) {
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        return result;
    }
    return ESP_OK;
}

esp_err_t MqttDriver::start()
{
    if (client_ == nullptr || started_) return ESP_ERR_INVALID_STATE;
    connecting_.store(true);
    const esp_err_t result = esp_mqtt_client_start(client_);
    if (result == ESP_OK) started_ = true;
    else connecting_.store(false);
    return result;
}

esp_err_t MqttDriver::reconnect()
{
    if (client_ == nullptr || !started_) return ESP_ERR_INVALID_STATE;
    connecting_.store(true);
    const esp_err_t result = esp_mqtt_client_reconnect(client_);
    if (result != ESP_OK) connecting_.store(false);
    return result;
}

esp_err_t MqttDriver::disconnect()
{
    if (client_ == nullptr || !started_) return ESP_OK;
    connected_.store(false);
    connecting_.store(false);
    return esp_mqtt_client_disconnect(client_);
}

esp_err_t MqttDriver::enqueueJson(const char* topic, const char* payload,
                                  size_t payload_length, int qos, bool retain)
{
    if (client_ == nullptr || !connected_.load()) return ESP_ERR_INVALID_STATE;
    if (topic == nullptr || topic[0] == '\0' || payload == nullptr || payload_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int message_id = esp_mqtt_client_enqueue(client_, topic, payload,
        static_cast<int>(payload_length), qos, retain, true);
    if (message_id >= 0) {
        ESP_LOGI(TAG,
                 "Publish accepted: id=%d topic=%s bytes=%u qos=%d",
                 message_id,
                 topic,
                 static_cast<unsigned>(payload_length),
                 qos);
        return ESP_OK;
    }
    if (message_id == -2) return ESP_ERR_NO_MEM;  // Internal MQTT outbox is full.
    return ESP_FAIL;
}

bool MqttDriver::isConnected() const { return connected_.load(); }
bool MqttDriver::isConnecting() const { return connecting_.load(); }

void MqttDriver::eventHandler(void* handler_argument, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    (void)event_base;
    (void)event_data;
    auto* self = static_cast<MqttDriver*>(handler_argument);
    if (self != nullptr) self->handleEvent(event_id);
}

void MqttDriver::handleEvent(int32_t event_id)
{
    if (event_id == MQTT_EVENT_BEFORE_CONNECT) {
        connecting_.store(true);
    } else if (event_id == MQTT_EVENT_CONNECTED) {
        connected_.store(true);
        connecting_.store(false);
        ESP_LOGI(TAG, "Connected to broker");
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        connected_.store(false);
        connecting_.store(false);
        ESP_LOGW(TAG, "Disconnected from broker");
    } else if (event_id == MQTT_EVENT_ERROR) {
        connected_.store(false);
        connecting_.store(false);
        ESP_LOGE(TAG, "MQTT transport error");
    }
    // Inbound MQTT_DATA is intentionally not handled in this outbound-only phase.
}

}  // namespace minicar::driver
