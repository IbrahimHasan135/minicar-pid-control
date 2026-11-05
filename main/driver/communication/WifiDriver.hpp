#pragma once

#include <atomic>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"

namespace minicar::driver {

class WifiDriver {
public:
    esp_err_t init(const char* ssid, const char* password,
                   const char* ap_ssid, const char* ap_password);
    esp_err_t connect();
    bool isConnected() const;
    bool isConnecting() const;

private:
    static void eventHandler(void* argument, esp_event_base_t event_base,
                             int32_t event_id, void* event_data);
    void handleEvent(esp_event_base_t event_base, int32_t event_id);

    esp_netif_t* netif_{nullptr};
    esp_netif_t* ap_netif_{nullptr};
    esp_event_handler_instance_t wifi_event_instance_{nullptr};
    esp_event_handler_instance_t ip_event_instance_{nullptr};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> connecting_{false};
};

}  // namespace minicar::driver
