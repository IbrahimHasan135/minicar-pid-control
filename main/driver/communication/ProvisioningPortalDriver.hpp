#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "model/NetworkSettings.hpp"
#include "service/configuration/NetworkSettingsService.hpp"

namespace minicar::driver {

class ProvisioningPortalDriver {
public:
    esp_err_t start(service::NetworkSettingsService& settings_service);

private:
    static esp_err_t rootHandler(httpd_req_t* request);
    static esp_err_t saveHandler(httpd_req_t* request);
    static esp_err_t redirectHandler(httpd_req_t* request);
    esp_err_t handleRoot(httpd_req_t* request) const;
    esp_err_t handleSave(httpd_req_t* request);

    service::NetworkSettingsService* settings_service_{nullptr};
    httpd_handle_t server_{nullptr};
};

}  // namespace minicar::driver
