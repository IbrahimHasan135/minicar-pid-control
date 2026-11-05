#pragma once

#include "driver/communication/WifiDriver.hpp"
#include "esp_err.h"

namespace minicar::service {

class WifiService {
public:
    WifiService(driver::WifiDriver& driver, const char* ssid, const char* password,
                const char* ap_ssid, const char* ap_password);

    bool isConfigured() const;
    esp_err_t init();
    esp_err_t connect();
    bool isConnected() const;
    bool isConnecting() const;

private:
    driver::WifiDriver& driver_;
    const char* ssid_;
    const char* password_;
    const char* ap_ssid_;
    const char* ap_password_;
};

}  // namespace minicar::service
