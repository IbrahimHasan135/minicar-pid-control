#include "service/communication/WifiService.hpp"

namespace minicar::service {

WifiService::WifiService(driver::WifiDriver& driver, const char* ssid, const char* password,
                         const char* ap_ssid, const char* ap_password)
    : driver_(driver), ssid_(ssid), password_(password),
      ap_ssid_(ap_ssid), ap_password_(ap_password)
{
}

bool WifiService::isConfigured() const
{
    return ssid_ != nullptr && password_ != nullptr && ssid_[0] != '\0';
}

esp_err_t WifiService::init()
{
    return driver_.init(ssid_, password_, ap_ssid_, ap_password_);
}

esp_err_t WifiService::connect() { return driver_.connect(); }
bool WifiService::isConnected() const { return driver_.isConnected(); }
bool WifiService::isConnecting() const { return driver_.isConnecting(); }

}  // namespace minicar::service
