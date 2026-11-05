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

esp_err_t WifiDriver::init(const char* ssid, const char* password,
                           const char* ap_ssid, const char* ap_password)
{
    if (ssid == nullptr || password == nullptr || ap_ssid == nullptr ||
        ap_password == nullptr || ap_ssid[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (initialized_.load()) return ESP_OK;

    wifi_config_t sta_config{};
    wifi_config_t ap_config{};
    const size_t ssid_length = std::strlen(ssid);
    const size_t password_length = std::strlen(password);
    const size_t ap_ssid_length = std::strlen(ap_ssid);
    const size_t ap_password_length = std::strlen(ap_password);
    if (ssid_length >= sizeof(sta_config.sta.ssid) ||
        password_length >= sizeof(sta_config.sta.password) ||
        ap_ssid_length >= sizeof(ap_config.ap.ssid) ||
        ap_password_length >= sizeof(ap_config.ap.password) ||
        (ap_password_length > 0 && ap_password_length < 8)) return ESP_ERR_INVALID_SIZE;

    esp_err_t result = initNvs();
    if (result != ESP_OK) return result;

    result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;
    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;

    netif_ = esp_netif_create_default_wifi_sta();
    ap_netif_ = esp_netif_create_default_wifi_ap();
    if (netif_ == nullptr || ap_netif_ == nullptr) return ESP_FAIL;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&init_config);
    if (result != ESP_OK) return result;

    result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &WifiDriver::eventHandler, this, &wifi_event_instance_);
    if (result != ESP_OK) return result;
    result = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &WifiDriver::eventHandler, this, &ip_event_instance_);
    if (result != ESP_OK) return result;

    std::memcpy(sta_config.sta.ssid, ssid, ssid_length);
    std::memcpy(sta_config.sta.password, password, password_length);
    std::memcpy(ap_config.ap.ssid, ap_ssid, ap_ssid_length);
    std::memcpy(ap_config.ap.password, ap_password, ap_password_length);
    ap_config.ap.ssid_len = static_cast<uint8_t>(ap_ssid_length);
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = ap_password_length == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    result = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (result != ESP_OK) return result;
    result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (result != ESP_OK) return result;
    if (ssid_length > 0) result = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (result != ESP_OK) return result;
    result = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (result != ESP_OK) return result;
    result = esp_wifi_start();
    if (result != ESP_OK) return result;

    initialized_.store(true);
    ESP_LOGI(TAG, "Wi-Fi AP+STA initialized; setup SSID=%s", ap_ssid);
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
