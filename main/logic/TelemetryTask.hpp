#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "model/TelemetryMessage.hpp"
#include "service/telemetry/TelemetryService.hpp"

namespace minicar::logic {

class TelemetryTask {
public:
    TelemetryTask(QueueHandle_t telemetry_queue, service::TelemetryService& telemetry_service);

    BaseType_t start();

private:
    static void taskEntry(void* context);
    void run();

    QueueHandle_t telemetry_queue_{nullptr};
    service::TelemetryService& telemetry_service_;
};

}  // namespace minicar::logic
