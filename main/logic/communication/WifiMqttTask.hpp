#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logic/communication/MqttPublishEventQueue.hpp"
#include "service/communication/MqttCommunicationService.hpp"
#include "service/communication/WifiService.hpp"

namespace minicar::logic {

// Owns connection watchdog/retry timing and drains outbound JSON publish events.
class WifiMqttTask {
public:
    WifiMqttTask(MqttPublishEventQueue& publish_queue,
                 service::WifiService& wifi_service,
                 service::MqttCommunicationService& mqtt_service);

    BaseType_t start();

private:
    static void taskEntry(void* context);
    void run();

    MqttPublishEventQueue& publish_queue_;
    service::WifiService& wifi_service_;
    service::MqttCommunicationService& mqtt_service_;
};

}  // namespace minicar::logic
