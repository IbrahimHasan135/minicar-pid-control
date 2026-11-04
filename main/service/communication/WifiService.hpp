#pragma once

#include "driver/communication/WifiDriver.hpp"
#include "esp_err.h"

namespace minicar::service {

class WifiService {
public:
    WifiService(driver::WifiDriver& driver, const char* ssid, const char* password);

    bool isConfigured() const;
    esp_err_t init();
    esp_err_t connect();
    bool isConnected() const;
    bool isConnecting() const;

private:
    driver::WifiDriver& driver_;
    const char* ssid_;
    const char* password_;
};

}  // namespace minicar::service
