#include "logic/TelemetryTask.hpp"

#include "config/FreeRTOSConfig.hpp"

namespace minicar::logic {

TelemetryTask::TelemetryTask(QueueHandle_t telemetry_queue, service::TelemetryService& telemetry_service)
    : telemetry_queue_(telemetry_queue), telemetry_service_(telemetry_service)
{
}

BaseType_t TelemetryTask::start()
{
    return xTaskCreate(
        &TelemetryTask::taskEntry,
        "TelemetryTask",
        config::rtos::TELEMETRY_TASK_STACK_WORDS,
        this,
        config::rtos::TELEMETRY_TASK_PRIORITY,
        nullptr);
}

void TelemetryTask::taskEntry(void* context)
{
    static_cast<TelemetryTask*>(context)->run();
}

void TelemetryTask::run()
{
    model::TelemetryMessage message{};

    while (true) {
        if (xQueueReceive(telemetry_queue_, &message, portMAX_DELAY) == pdTRUE) {
            (void)telemetry_service_.publish(message);
        }
    }
}

}  // namespace minicar::logic
