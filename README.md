# minicar-pid-control

## Wi-Fi/MQTT outbound queue

The `feat/wifi-mqtt-com` implementation adds an outbound-only Wi-Fi/MQTT connection task. Other tasks can enqueue topic + JSON payloads with `logic::MqttPublishEventQueue::pushJson(...)`; `WifiMqttTask` watches Wi-Fi/MQTT connectivity and publishes queued events when online. No MQTT subscriptions or inbound command handling are enabled.

Wi-Fi SSID/password and broker URI/credentials are intentionally empty in `main/config/NetworkConfig.hpp`. Until configured, network publishing stays idle and the bounded queue can fill; producer calls return `false` when an event cannot be queued. See [docs/WIFI_MQTT_COMMUNICATION_PLAN.md](docs/WIFI_MQTT_COMMUNICATION_PLAN.md) for the architecture and queue contract.

