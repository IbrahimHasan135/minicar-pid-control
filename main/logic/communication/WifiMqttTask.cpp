#include "logic/communication/WifiMqttTask.hpp"

#include "config/NetworkConfig.hpp"
#include "esp_log.h"

namespace minicar::logic {
namespace {
constexpr char TAG[] = "WifiMqttTask";

bool elapsed(TickType_t now, TickType_t previous, uint32_t period_ms)
{
    return (now - previous) >= pdMS_TO_TICKS(period_ms);
}
}  // namespace

WifiMqttTask::WifiMqttTask(MqttPublishEventQueue& publish_queue,
                           service::WifiService& wifi_service,
                           service::MqttCommunicationService& mqtt_service)
    : publish_queue_(publish_queue), wifi_service_(wifi_service), mqtt_service_(mqtt_service)
{
}

BaseType_t WifiMqttTask::start()
{
    if (!publish_queue_.isReady()) return pdFAIL;
    return xTaskCreate(&WifiMqttTask::taskEntry, "WifiMqttTask",
        config::network::WIFI_MQTT_TASK_STACK_WORDS, this,
        config::network::WIFI_MQTT_TASK_PRIORITY, nullptr);
}

void WifiMqttTask::taskEntry(void* context)
{
    static_cast<WifiMqttTask*>(context)->run();
}

void WifiMqttTask::run()
{
    if (!wifi_service_.isConfigured()) {
        ESP_LOGW(TAG, "Wi-Fi SSID is blank; task is idle and publish events remain queued");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    const esp_err_t wifi_init_result = wifi_service_.init();
    if (wifi_init_result != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed: %s", esp_err_to_name(wifi_init_result));
        vTaskDelete(nullptr);
        return;
    }

    const bool mqtt_configured = mqtt_service_.isConfigured();
    bool mqtt_ready = false;
    if (mqtt_configured) {
        const esp_err_t mqtt_init_result = mqtt_service_.init();
        mqtt_ready = mqtt_init_result == ESP_OK;
        if (!mqtt_ready) ESP_LOGE(TAG, "MQTT initialization failed: %s", esp_err_to_name(mqtt_init_result));
    } else {
        ESP_LOGW(TAG, "MQTT broker URI is blank; Wi-Fi watchdog runs, publish events remain queued");
    }

    TickType_t last_wifi_attempt = 0;
    TickType_t last_mqtt_attempt = 0;
    bool wifi_attempted = false;
    bool mqtt_attempted = false;
    bool mqtt_started = false;
    bool wifi_was_connected = false;
    bool has_pending_event = false;
    model::MqttPublishEvent pending_event{};

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        const bool wifi_connected = wifi_service_.isConnected();

        if (!wifi_connected) {
            if (wifi_was_connected && mqtt_ready) {
                (void)mqtt_service_.disconnect();
                ESP_LOGW(TAG, "Wi-Fi lost; MQTT publish paused");
            }
            wifi_was_connected = false;

            if (!wifi_service_.isConnecting() &&
                (!wifi_attempted || elapsed(now, last_wifi_attempt, config::network::WIFI_RETRY_PERIOD_MS))) {
                const esp_err_t result = wifi_service_.connect();
                last_wifi_attempt = now;
                wifi_attempted = true;
                if (result != ESP_OK) ESP_LOGW(TAG, "Wi-Fi connect request failed: %s", esp_err_to_name(result));
                else ESP_LOGI(TAG, "Wi-Fi connection requested");
            }
            vTaskDelay(pdMS_TO_TICKS(config::network::CONNECTION_TASK_WAKE_PERIOD_MS));
            continue;
        }

        if (!wifi_was_connected) ESP_LOGI(TAG, "Wi-Fi connected; MQTT watchdog enabled");
        wifi_was_connected = true;

        if (mqtt_ready && !mqtt_service_.isConnected() && !mqtt_service_.isConnecting()) {
            const bool retry_due = !mqtt_attempted ||
                elapsed(now, last_mqtt_attempt, config::network::MQTT_RETRY_PERIOD_MS);
            if (retry_due) {
                const esp_err_t result = mqtt_started ? mqtt_service_.reconnect() : mqtt_service_.start();
                last_mqtt_attempt = now;
                mqtt_attempted = true;
                if (result == ESP_OK) {
                    mqtt_started = true;
                    ESP_LOGI(TAG, "MQTT connection requested");
                } else {
                    ESP_LOGW(TAG, "MQTT connection request failed: %s", esp_err_to_name(result));
                }
            }
        }

        if (mqtt_ready && mqtt_service_.isConnected()) {
            if (!has_pending_event) has_pending_event = publish_queue_.pull(pending_event, 0);
            if (has_pending_event) {
                const esp_err_t result = mqtt_service_.publish(pending_event);
                if (result == ESP_OK) {
                    has_pending_event = false;  // Accepted into ESP-MQTT's bounded outbox.
                } else if (result == ESP_ERR_INVALID_ARG) {
                    ESP_LOGE(TAG, "Dropped invalid MQTT publish event");
                    has_pending_event = false;
                } else {
                    ESP_LOGW(TAG, "MQTT publish deferred: %s", esp_err_to_name(result));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(config::network::CONNECTION_TASK_WAKE_PERIOD_MS));
    }
}

}  // namespace minicar::logic
