# Wi-Fi and MQTT Outbound Communication

Status: **outbound queue and connection watchdog implemented; credentials intentionally blank**  
Branch: `feat/wifi-mqtt-com`  
Target: ESP-IDF 5.4.4, ESP32 DevKit V1, C++, FreeRTOS

This layer keeps Wi-Fi/MQTT lifecycle and publishing separate from motion control. It only publishes JSON events submitted by other tasks. MQTT command reception, subscription, and command parsing are outside this phase.

## Layer ownership

```text
Application / app_main
    constructs drivers, services, event queue, and WifiMqttTask

Logic
    MqttPublishEventQueue: bounded push/pull queue for outbound JSON events
    WifiMqttTask: Wi-Fi/MQTT watchdog, retry cadence, and queue consumer

Service
    WifiService: Wi-Fi station capability
    MqttCommunicationService: MQTT lifecycle request and JSON publish capability

Driver
    WifiDriver: ESP-IDF Wi-Fi/netif calls and connection state
    MqttDriver: ESP-MQTT client lifecycle and non-blocking enqueue
```

Dependency construction stays in `app_main()`. A message producer receives an `MqttPublishEventQueue&` and calls its push API. Producers do not access services or drivers. `WifiMqttTask` pulls only while Wi-Fi and MQTT are connected, then passes the event to `MqttCommunicationService`. Services own their matching driver; FreeRTOS queue operations remain in Logic.

## Outbound event contract

`model::MqttPublishEvent` is a fixed-size queue item with:

- `topic[96]`
- `json_payload[512]`

The queue has 8 slots. A producer enqueues with:

```cpp
const bool queued = mqtt_publish_queue.pushJson(
    "minicar/demo/telemetry",
    R"({"kind":"sample","value":12.5})",
    0);  // no wait; false means invalid input or a full queue
```

The payload is forwarded unchanged; the communication layer does not interpret or build JSON. Producers should treat a `false` return as a dropped event and decide whether to count, retry, or discard it. Keep the payload shorter than 512 bytes and the topic shorter than 96 bytes, including the terminating NUL requirement.

When disconnected, the task does not drain the queue. It keeps at most one already-pulled event as a pending item if the MQTT outbox rejects it, while newer events remain in the bounded FreeRTOS queue. This preserves FIFO ordering and bounds RAM usage. A full queue applies backpressure through `pushJson()` returning `false`; the task never blocks the motion loop.

The MQTT driver uses `esp_mqtt_client_enqueue()` at QoS 1 with retain disabled. This copies accepted messages to ESP-MQTT's internal outbox for asynchronous transmission. The FreeRTOS event is released once accepted by that outbox; it is not a persistent disk queue, so power loss can discard queued data.

## Connection watchdog

`WifiMqttTask` runs independently of `MotionLoopTask` and checks connectivity every 100 ms:

1. If Wi-Fi credentials are present, it initializes the station and requests a connection when disconnected/not connecting. Retries are spaced by 5 seconds.
2. It initializes MQTT only when a broker URI is configured, and starts MQTT only after Wi-Fi has received an IP address.
3. MQTT automatic reconnect is disabled so the task owns retry timing. It requests reconnect every 5 seconds while Wi-Fi is available and MQTT is disconnected.
4. If Wi-Fi drops, MQTT publishing pauses. The current event and queued events are retained in RAM until reconnect or queue capacity is reached.
5. If either configuration is blank, the relevant network path stays inactive; motion and telemetry tasks continue.

`WifiDriver` stores Wi-Fi credentials in RAM and does not erase NVS. Broker scheme/port and TLS settings will be selected when a real broker is provided. Credentials are not logged.

## Configuration

The initial placeholders are in `main/config/NetworkConfig.hpp` and are intentionally empty:

```cpp
WIFI_SSID = ""
WIFI_PASSWORD = ""
MQTT_BROKER_URI = ""
MQTT_USERNAME = ""
MQTT_PASSWORD = ""
```

No broker or Wi-Fi connection is attempted until the corresponding values are filled. Client ID is also blank, allowing ESP-MQTT's default ID. The queue and connection task are still created, so other tasks can be wired to the event queue without coupling them to network availability.

## Current scope and non-goals

- Outbound publish only. There is no MQTT subscribe call, `MQTT_EVENT_DATA` handling, command decoder, or remote motor control.
- Existing serial `CommunicationService` and `SerialTask` remain separate and unchanged.
- Existing `TelemetryQueue → TelemetryTask → TelemetryService → ConsoleDriver` remains unchanged. Telemetry is not automatically sent to MQTT; a producer must explicitly call `mqtt_publish_queue.pushJson(...)`.
- No Wi-Fi or MQTT secrets, URI, topic convention, or TLS certificate has been selected.
- No changes to motion PID, odometry, motor/IMU services, or localization.

## Files added/changed

- `main/config/NetworkConfig.hpp`: blank credentials, queue/task sizing, retry periods, QoS.
- `main/model/MqttPublishEvent.hpp`: fixed-size publish event.
- `main/logic/communication/MqttPublishEventQueue.*`: bounded event queue and producer API.
- `main/logic/communication/WifiMqttTask.*`: connection watchdog and queue consumer.
- `main/driver/communication/WifiDriver.*` and `MqttDriver.*`: ESP-IDF/ESP-MQTT adapters.
- `main/service/communication/WifiService.*` and `MqttCommunicationService.*`: capability APIs.
- `main/app_main.cpp`, `main/CMakeLists.txt`: dependency injection, task startup, and component dependencies.

ESP-MQTT is used through the ESP-IDF 5.4.4 `mqtt` component; the component requirements are listed in `main/CMakeLists.txt`.

## Next configuration decisions

Before using the network path, provide:

1. Wi-Fi SSID and password, plus the intended provisioning method.
2. MQTT URI, authentication, and transport/TLS requirements.
3. Topic names for each producer and whether any event needs a different QoS or retain setting.
4. A policy for each producer when `pushJson()` returns `false` because the bounded queue is full.

## References

- [ESP-IDF v5.4.4 Wi-Fi Driver guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-guides/wifi.html)
- [ESP-IDF v5.4.4 ESP-MQTT guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/protocols/mqtt.html)
