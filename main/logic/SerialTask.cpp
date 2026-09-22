#include "logic/SerialTask.hpp"

#include "config/FreeRTOSConfig.hpp"

namespace minicar::logic {

SerialTask::SerialTask(QueueHandle_t motion_queue, service::CommunicationService& communication_service)
    : motion_queue_(motion_queue), communication_service_(communication_service)
{
}

BaseType_t SerialTask::start()
{
    return xTaskCreate(
        &SerialTask::taskEntry,
        "SerialTask",
        config::rtos::SERIAL_TASK_STACK_WORDS,
        this,
        config::rtos::SERIAL_TASK_PRIORITY,
        nullptr);
}

void SerialTask::taskEntry(void* context)
{
    static_cast<SerialTask*>(context)->run();
}

void SerialTask::run()
{
    model::MotionCommand command{};

    while (true) {
        if (communication_service_.readCommand(command, config::rtos::SERIAL_TASK_IDLE_DELAY_MS)) {
            (void)xQueueSend(motion_queue_, &command, 0);
        } else {
            vTaskDelay(pdMS_TO_TICKS(config::rtos::SERIAL_TASK_IDLE_DELAY_MS));
        }
    }
}

}  // namespace minicar::logic
