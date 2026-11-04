#include "driver/communication/WifiDriver.hpp"

#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

namespace minicar::driver {
namespace {
constexpr char TAG[] = "WifiDriver";

esp_err_t initNvs()
{
    // Do not erase NVS from a connectivity driver; return storage errors to the caller.
    return nvs_flash_init();
}
}  // namespace

esp_err_t WifiDriver::init(const char* ssid, const char* password)
{
    if (ssid == nullptr || password == nullptr || ssid[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (initialized_.load()) return ESP_OK;

    wifi_config_t wifi_config{};
    const size_t ssid_length = std::strlen(ssid);
    const size_t password_length = std::strlen(password);
    if (ssid_length >= sizeof(wifi_config.sta.ssid) ||
        password_length >= sizeof(wifi_config.sta.password)) return ESP_ERR_INVALID_SIZE;

    esp_err_t result = initNvs();
    if (result != ESP_OK) return result;

    result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;
    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;

    netif_ = esp_netif_create_default_wifi_sta();
    if (netif_ == nullptr) return ESP_FAIL;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&init_config);
    if (result != ESP_OK) return result;

    result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &WifiDriver::eventHandler, this, &wifi_event_instance_);
    if (result != ESP_OK) return result;
    result = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &WifiDriver::eventHandler, this, &ip_event_instance_);
    if (result != ESP_OK) return result;

    std::memcpy(wifi_config.sta.ssid, ssid, ssid_length);
    std::memcpy(wifi_config.sta.password, password, password_length);
    result = esp_wifi_set_mode(WIFI_MODE_STA);
    if (result != ESP_OK) return result;
    result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (result != ESP_OK) return result;
    result = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (result != ESP_OK) return result;
    result = esp_wifi_start();
    if (result != ESP_OK) return result;

    initialized_.store(true);
    ESP_LOGI(TAG, "Wi-Fi station initialized");
    return ESP_OK;
}

esp_err_t WifiDriver::connect()
{
    if (!initialized_.load()) return ESP_ERR_INVALID_STATE;
    if (connected_.load() || connecting_.load()) return ESP_OK;
    const esp_err_t result = esp_wifi_connect();
    if (result == ESP_OK) connecting_.store(true);
    return result;
}

bool WifiDriver::isConnected() const { return connected_.load(); }
bool WifiDriver::isConnecting() const { return connecting_.load(); }

void WifiDriver::eventHandler(void* argument, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    (void)event_data;
    auto* self = static_cast<WifiDriver*>(argument);
    if (self != nullptr) self->handleEvent(event_base, event_id);
}

void WifiDriver::handleEvent(esp_event_base_t event_base, int32_t event_id)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        connected_.store(true);
        connecting_.store(false);
        ESP_LOGI(TAG, "Wi-Fi station received an IP address");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        connected_.store(false);
        connecting_.store(false);
        ESP_LOGW(TAG, "Wi-Fi station disconnected");
    }
}

}  // namespace minicar::driver
