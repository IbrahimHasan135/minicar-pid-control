#include "service/configuration/NetworkSettingsService.hpp"

#include <cstdint>
#include <cstring>

#include "config/NetworkConfig.hpp"
#include "nvs.h"
#include "nvs_flash.h"

namespace minicar::service {
namespace {

constexpr char NAMESPACE[] = "network-config";

template <std::size_t N>
void copyText(char (&destination)[N], const char* source)
{
    if (source == nullptr) source = "";
    std::strncpy(destination, source, N - 1);
    destination[N - 1] = '\0';
}

void readText(nvs_handle_t handle, const char* key, char* output,
              std::size_t capacity, const char* fallback)
{
    std::size_t required = capacity;
    if (nvs_get_str(handle, key, output, &required) != ESP_OK) {
        if (output != fallback) {
            std::strncpy(output, fallback != nullptr ? fallback : "", capacity - 1);
            output[capacity - 1] = '\0';
        }
    }
}

esp_err_t writeText(nvs_handle_t handle, const char* key, const char* value)
{
    return nvs_set_str(handle, key, value != nullptr ? value : "");
}

}  // namespace

esp_err_t NetworkSettingsService::init()
{
    esp_err_t result = nvs_flash_init();
    if (result != ESP_OK) return result;

    copyText(settings_.wifi_ssid, config::network::WIFI_SSID);
    copyText(settings_.wifi_password, config::network::WIFI_PASSWORD);
    copyText(settings_.mqtt_uri, config::network::MQTT_BROKER_URI);
    copyText(settings_.mqtt_username, config::network::MQTT_USERNAME);
    copyText(settings_.mqtt_password, config::network::MQTT_PASSWORD);
    copyText(settings_.mqtt_client_id, config::network::MQTT_CLIENT_ID);
    copyText(settings_.mdns_hostname, config::network::MDNS_HOSTNAME);

    nvs_handle_t handle{};
    result = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (result != ESP_OK) return result;
    readText(handle, "wifi_ssid", settings_.wifi_ssid, sizeof(settings_.wifi_ssid), settings_.wifi_ssid);
    readText(handle, "wifi_pass", settings_.wifi_password, sizeof(settings_.wifi_password), settings_.wifi_password);
    readText(handle, "mqtt_uri", settings_.mqtt_uri, sizeof(settings_.mqtt_uri), settings_.mqtt_uri);
    readText(handle, "mqtt_user", settings_.mqtt_username, sizeof(settings_.mqtt_username), settings_.mqtt_username);
    readText(handle, "mqtt_pass", settings_.mqtt_password, sizeof(settings_.mqtt_password), settings_.mqtt_password);
    readText(handle, "mqtt_client", settings_.mqtt_client_id, sizeof(settings_.mqtt_client_id), settings_.mqtt_client_id);
    readText(handle, "mdns_host", settings_.mdns_hostname, sizeof(settings_.mdns_hostname), settings_.mdns_hostname);
    nvs_close(handle);
    return ESP_OK;
}

const model::NetworkSettings& NetworkSettingsService::settings() const { return settings_; }

esp_err_t NetworkSettingsService::save(const model::NetworkSettings& settings)
{
    nvs_handle_t handle{};
    esp_err_t result = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;

    const struct { const char* key; const char* value; } values[] = {
        {"wifi_ssid", settings.wifi_ssid}, {"wifi_pass", settings.wifi_password},
        {"mqtt_uri", settings.mqtt_uri}, {"mqtt_user", settings.mqtt_username},
        {"mqtt_pass", settings.mqtt_password}, {"mqtt_client", settings.mqtt_client_id},
        {"mdns_host", settings.mdns_hostname},
    };
    for (const auto& entry : values) {
        result = writeText(handle, entry.key, entry.value);
        if (result != ESP_OK) break;
    }
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    if (result == ESP_OK) settings_ = settings;
    return result;
}

}  // namespace minicar::service
