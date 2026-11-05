#include "driver/communication/ProvisioningPortalDriver.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>

#include "config/NetworkConfig.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdns.h"

namespace minicar::driver {
namespace {

constexpr char TAG[] = "ProvisioningPortal";

constexpr char PAGE[] = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NovaLynk Minicar Setup</title><style>
body{font-family:system-ui;background:#f4f6f8;margin:0;padding:18px;color:#17202a}
.card{max-width:680px;margin:auto;background:white;padding:20px;border-radius:14px}
h1{margin-top:0}label{display:block;font-weight:600;margin-top:14px}
input{width:100%;box-sizing:border-box;padding:11px;border:1px solid #bbc3cc;border-radius:8px}
.hint{font-size:12px;color:#68737d}.status{background:#eef7ff;padding:12px;border-radius:8px}
button{width:100%;margin-top:20px;padding:12px;border:0;border-radius:8px;background:#145da0;color:white;font-weight:700}
</style></head><body><div class="card"><h1>NovaLynk Minicar</h1>
<div class="status">AP: <b>NovaLynk-Minicar-Setup</b><br>IP: <b>192.168.4.1</b><br>
mDNS: <b>http://novalynk-minicar.local</b></div>
<form method="post" action="/save"><label>Wi-Fi SSID</label><input name="wifi_ssid" maxlength="32" required>
<label>Wi-Fi Password</label><input name="wifi_password" type="password" maxlength="64"><div class="hint">Kosongkan untuk mempertahankan password tersimpan.</div>
<label>MQTT Broker URI</label><input name="mqtt_uri" maxlength="159" placeholder="mqtts://host:8883" required>
<label>MQTT Username</label><input name="mqtt_username" maxlength="95">
<label>MQTT Password</label><input name="mqtt_password" type="password" maxlength="127"><div class="hint">Kosongkan untuk mempertahankan password tersimpan.</div>
<label>MQTT Client ID</label><input name="mqtt_client_id" maxlength="63" value="minicar-arand-test">
<label>mDNS Hostname</label><input name="mdns_hostname" maxlength="63" value="novalynk-minicar">
<button type="submit">Simpan dan Restart</button></form></div></body></html>)HTML";

void urlDecode(char* text)
{
    char* output = text;
    for (char* input = text; *input != '\0'; ++input) {
        if (*input == '+') {
            *output++ = ' ';
        } else if (*input == '%' && std::isxdigit(static_cast<unsigned char>(input[1])) &&
                   std::isxdigit(static_cast<unsigned char>(input[2]))) {
            unsigned int value = 0;
            std::sscanf(input + 1, "%2x", &value);
            *output++ = static_cast<char>(value);
            input += 2;
        } else {
            *output++ = *input;
        }
    }
    *output = '\0';
}

bool readField(const char* body, const char* key, char* output, std::size_t capacity)
{
    if (httpd_query_key_value(body, key, output, capacity) != ESP_OK) return false;
    urlDecode(output);
    return true;
}

bool validHostname(const char* hostname)
{
    if (hostname == nullptr || hostname[0] == '\0') return false;
    for (const char* c = hostname; *c != '\0'; ++c) {
        if (!(std::isalnum(static_cast<unsigned char>(*c)) || *c == '-')) return false;
    }
    return true;
}

}  // namespace

esp_err_t ProvisioningPortalDriver::start(service::NetworkSettingsService& settings_service)
{
    settings_service_ = &settings_service;
    if (mdns_init() != ESP_OK ||
        mdns_hostname_set(settings_service.settings().mdns_hostname) != ESP_OK) {
        ESP_LOGW(TAG, "mDNS initialization failed; use http://192.168.4.1");
    } else {
        (void)mdns_instance_name_set("NovaLynk Minicar");
        (void)mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.uri_match_fn = httpd_uri_match_wildcard;
    esp_err_t result = httpd_start(&server_, &config);
    if (result != ESP_OK) return result;

    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = &rootHandler, .user_ctx = this},
        {.uri = "/save", .method = HTTP_POST, .handler = &saveHandler, .user_ctx = this},
        {.uri = "/*", .method = HTTP_GET, .handler = &redirectHandler, .user_ctx = this},
    };
    for (const auto& route : routes) {
        result = httpd_register_uri_handler(server_, &route);
        if (result != ESP_OK) return result;
    }
    ESP_LOGI(TAG, "setup portal ready: SSID=%s URL=http://%s.local",
             config::network::PROVISIONING_AP_SSID,
             settings_service.settings().mdns_hostname);
    return ESP_OK;
}

esp_err_t ProvisioningPortalDriver::rootHandler(httpd_req_t* request)
{
    return static_cast<ProvisioningPortalDriver*>(request->user_ctx)->handleRoot(request);
}

esp_err_t ProvisioningPortalDriver::saveHandler(httpd_req_t* request)
{
    return static_cast<ProvisioningPortalDriver*>(request->user_ctx)->handleSave(request);
}

esp_err_t ProvisioningPortalDriver::redirectHandler(httpd_req_t* request)
{
    httpd_resp_set_status(request, "302 Found");
    httpd_resp_set_hdr(request, "Location", "/");
    return httpd_resp_send(request, nullptr, 0);
}

esp_err_t ProvisioningPortalDriver::handleRoot(httpd_req_t* request) const
{
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_send(request, PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t ProvisioningPortalDriver::handleSave(httpd_req_t* request)
{
    if (request->content_len <= 0 || request->content_len > 1024) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid form size");
    }
    char body[1025]{};
    int received = 0;
    while (received < request->content_len) {
        const int count = httpd_req_recv(request, body + received, request->content_len - received);
        if (count <= 0) return ESP_FAIL;
        received += count;
    }

    model::NetworkSettings updated = settings_service_->settings();
    char value[model::NetworkSettings::MQTT_URI_CAPACITY]{};
    if (readField(body, "wifi_ssid", value, sizeof(value))) {
        std::strncpy(updated.wifi_ssid, value, sizeof(updated.wifi_ssid) - 1);
    }
    if (readField(body, "wifi_password", value, sizeof(value)) && value[0] != '\0') {
        std::strncpy(updated.wifi_password, value, sizeof(updated.wifi_password) - 1);
    }
    if (readField(body, "mqtt_uri", value, sizeof(value))) {
        std::strncpy(updated.mqtt_uri, value, sizeof(updated.mqtt_uri) - 1);
    }
    if (readField(body, "mqtt_username", value, sizeof(value))) {
        std::strncpy(updated.mqtt_username, value, sizeof(updated.mqtt_username) - 1);
    }
    if (readField(body, "mqtt_password", value, sizeof(value)) && value[0] != '\0') {
        std::strncpy(updated.mqtt_password, value, sizeof(updated.mqtt_password) - 1);
    }
    if (readField(body, "mqtt_client_id", value, sizeof(value))) {
        std::strncpy(updated.mqtt_client_id, value, sizeof(updated.mqtt_client_id) - 1);
    }
    if (readField(body, "mdns_hostname", value, sizeof(value))) {
        if (!validHostname(value)) {
            return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid mDNS hostname");
        }
        std::strncpy(updated.mdns_hostname, value, sizeof(updated.mdns_hostname) - 1);
    }

    const esp_err_t result = settings_service_->save(updated);
    if (result != ESP_OK) return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS save failed");
    httpd_resp_set_type(request, "text/html");
    (void)httpd_resp_sendstr(request, "<h2>Configuration saved. Minicar is restarting...</h2>");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

}  // namespace minicar::driver
