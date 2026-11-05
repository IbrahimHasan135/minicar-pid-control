#include "driver/communication/ProvisioningPortalDriver.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>

#include "config/NetworkConfig.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdns.h"

namespace minicar::driver {
namespace {

constexpr char TAG[] = "ProvisioningPortal";

void appendEscaped(std::string& output, const char* value)
{
    if (value == nullptr) return;
    for (const char* c = value; *c != '\0'; ++c) {
        switch (*c) {
            case '&': output += "&amp;"; break;
            case '<': output += "&lt;"; break;
            case '>': output += "&gt;"; break;
            case '"': output += "&quot;"; break;
            case '\'': output += "&#39;"; break;
            default: output += *c; break;
        }
    }
}

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
        {.uri = "/save-wifi", .method = HTTP_POST, .handler = &saveHandler, .user_ctx = this},
        {.uri = "/save-mqtt", .method = HTTP_POST, .handler = &saveHandler, .user_ctx = this},
        {.uri = "/save-mdns", .method = HTTP_POST, .handler = &saveHandler, .user_ctx = this},
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
    const model::NetworkSettings& settings = settings_service_->settings();
    std::string page;
    page.reserve(7600);
    page += R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NovaLynk Minicar</title><style>
*{box-sizing:border-box}body{font-family:system-ui;background:#eef2f6;margin:0;padding:18px;color:#17202a}.card{max-width:680px;margin:auto;background:#fff;padding:22px;border-radius:16px;box-shadow:0 8px 30px #18283818}
h1{margin:0 0 6px}.sub{color:#66717d;margin:0 0 18px}.status{background:#edf7ff;padding:14px;border-radius:10px;line-height:1.7}.actions{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-top:18px}
button{border:0;border-radius:9px;padding:12px;font-weight:700;cursor:pointer}.primary{background:#145da0;color:#fff}.secondary{background:#e8eef4;color:#17324d}.modal{display:none;position:fixed;inset:0;background:#071624aa;padding:18px;overflow:auto}
.modal.open{display:flex}.dialog{width:min(620px,100%);margin:auto;background:#fff;padding:20px;border-radius:14px}.dialog-head{display:flex;justify-content:space-between;align-items:center}.close{width:auto;background:#edf1f5;color:#334;padding:8px 12px}
label{display:block;font-weight:650;margin-top:13px}input{width:100%;padding:11px;border:1px solid #b8c2cc;border-radius:8px}.password{display:grid;grid-template-columns:1fr auto;gap:8px}.password button{background:#e8eef4;color:#17324d}.save{width:100%;margin-top:20px;background:#16834b;color:#fff}@media(max-width:520px){.actions{grid-template-columns:1fr}}
</style></head><body><main class="card"><h1>NovaLynk Minicar</h1><p class="sub">Network configuration portal</p><div class="status"><b>Access Point</b>: NovaLynk-Minicar-Setup<br><b>IP</b>: 192.168.4.1<br><b>mDNS</b>: http://)HTML";
    appendEscaped(page, settings.mdns_hostname);
    page += R"HTML(.local</div><div class="actions"><button class="primary" onclick="openModal('wifi')">Edit Wi-Fi</button><button class="secondary" onclick="openModal('mqtt')">Edit MQTT</button><button class="secondary" onclick="openModal('mdns')">Edit mDNS</button></div></main>
<section id="wifi" class="modal"><form class="dialog" method="post" action="/save-wifi"><div class="dialog-head"><h2>Edit Wi-Fi</h2><button type="button" class="close" onclick="closeModal('wifi')">Tutup</button></div><label>Wi-Fi SSID</label><input name="wifi_ssid" maxlength="32" required value=")HTML";
    page += '"'; appendEscaped(page, settings.wifi_ssid); page += R"HTML("><label>Wi-Fi Password</label><div class="password"><input id="wifiPass" name="wifi_password" type="password" maxlength="64" value=")HTML";
    page += '"'; appendEscaped(page, settings.wifi_password); page += R"HTML("><button type="button" onclick="togglePassword('wifiPass',this)">Lihat</button></div><button class="save" type="submit">Simpan Wi-Fi</button></form></section>
<section id="mqtt" class="modal"><form class="dialog" method="post" action="/save-mqtt"><div class="dialog-head"><h2>Edit MQTT</h2><button type="button" class="close" onclick="closeModal('mqtt')">Tutup</button></div><label>Broker URI</label><input name="mqtt_uri" maxlength="159" required value=")HTML";
    page += '"'; appendEscaped(page, settings.mqtt_uri); page += R"HTML("><label>Username</label><input name="mqtt_username" maxlength="95" value=")HTML";
    page += '"'; appendEscaped(page, settings.mqtt_username); page += R"HTML("><label>Password</label><div class="password"><input id="mqttPass" name="mqtt_password" type="password" maxlength="127" value=")HTML";
    page += '"'; appendEscaped(page, settings.mqtt_password); page += R"HTML("><button type="button" onclick="togglePassword('mqttPass',this)">Lihat</button></div><label>Client ID</label><input name="mqtt_client_id" maxlength="63" value=")HTML";
    page += '"'; appendEscaped(page, settings.mqtt_client_id); page += R"HTML("><button class="save" type="submit">Simpan MQTT</button></form></section>
<section id="mdns" class="modal"><form class="dialog" method="post" action="/save-mdns"><div class="dialog-head"><h2>Edit mDNS</h2><button type="button" class="close" onclick="closeModal('mdns')">Tutup</button></div><label>mDNS Hostname</label><input name="mdns_hostname" maxlength="63" required value=")HTML";
    page += '"'; appendEscaped(page, settings.mdns_hostname); page += R"HTML("><button class="save" type="submit">Simpan mDNS</button></form></section>
<script>function openModal(id){document.getElementById(id).classList.add('open')}function closeModal(id){document.getElementById(id).classList.remove('open')}function togglePassword(id,b){const i=document.getElementById(id),s=i.type==='password';i.type=s?'text':'password';b.textContent=s?'Sembunyikan':'Lihat'}document.querySelectorAll('.modal').forEach(m=>m.addEventListener('click',e=>{if(e.target===m)m.classList.remove('open')}));</script></body></html>)HTML";
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_send(request, page.c_str(), static_cast<ssize_t>(page.size()));
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

    const bool saving_wifi = std::strcmp(request->uri, "/save-wifi") == 0;
    const bool saving_mqtt = std::strcmp(request->uri, "/save-mqtt") == 0;
    const bool saving_mdns = std::strcmp(request->uri, "/save-mdns") == 0;
    if (!saving_wifi && !saving_mqtt && !saving_mdns) {
        return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "Unknown settings group");
    }
    model::NetworkSettings updated = settings_service_->settings();
    char value[model::NetworkSettings::MQTT_URI_CAPACITY]{};
    if (saving_wifi && readField(body, "wifi_ssid", value, sizeof(value))) {
        std::strncpy(updated.wifi_ssid, value, sizeof(updated.wifi_ssid) - 1);
    }
    if (saving_wifi && readField(body, "wifi_password", value, sizeof(value))) {
        std::strncpy(updated.wifi_password, value, sizeof(updated.wifi_password) - 1);
    }
    if (saving_mqtt && readField(body, "mqtt_uri", value, sizeof(value))) {
        std::strncpy(updated.mqtt_uri, value, sizeof(updated.mqtt_uri) - 1);
    }
    if (saving_mqtt && readField(body, "mqtt_username", value, sizeof(value))) {
        std::strncpy(updated.mqtt_username, value, sizeof(updated.mqtt_username) - 1);
    }
    if (saving_mqtt && readField(body, "mqtt_password", value, sizeof(value))) {
        std::strncpy(updated.mqtt_password, value, sizeof(updated.mqtt_password) - 1);
    }
    if (saving_mqtt && readField(body, "mqtt_client_id", value, sizeof(value))) {
        std::strncpy(updated.mqtt_client_id, value, sizeof(updated.mqtt_client_id) - 1);
    }
    if (saving_mdns && readField(body, "mdns_hostname", value, sizeof(value))) {
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
