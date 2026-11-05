#pragma once

#include <cstddef>

namespace minicar::model {

struct NetworkSettings {
    static constexpr std::size_t WIFI_SSID_CAPACITY = 33;
    static constexpr std::size_t WIFI_PASSWORD_CAPACITY = 65;
    static constexpr std::size_t MQTT_URI_CAPACITY = 160;
    static constexpr std::size_t MQTT_USERNAME_CAPACITY = 96;
    static constexpr std::size_t MQTT_PASSWORD_CAPACITY = 128;
    static constexpr std::size_t MQTT_CLIENT_ID_CAPACITY = 64;
    static constexpr std::size_t HOSTNAME_CAPACITY = 64;

    char wifi_ssid[WIFI_SSID_CAPACITY]{};
    char wifi_password[WIFI_PASSWORD_CAPACITY]{};
    char mqtt_uri[MQTT_URI_CAPACITY]{};
    char mqtt_username[MQTT_USERNAME_CAPACITY]{};
    char mqtt_password[MQTT_PASSWORD_CAPACITY]{};
    char mqtt_client_id[MQTT_CLIENT_ID_CAPACITY]{};
    char mdns_hostname[HOSTNAME_CAPACITY]{};
};

}  // namespace minicar::model
