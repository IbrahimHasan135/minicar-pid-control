#include "logic/motion_control/MotionCommandTask.hpp"

#include "config/FreeRTOSConfig.hpp"

namespace minicar::logic::motion_control {

MotionCommandTask::MotionCommandTask(QueueHandle_t motion_queue, MotionControlContext& context)
    : motion_queue_(motion_queue), context_(context)
{
}

BaseType_t MotionCommandTask::start()
{
    return xTaskCreate(
        &MotionCommandTask::taskEntry,
        "MotionCommandTask",
        config::rtos::MOTION_TASK_STACK_WORDS,
        this,
        config::rtos::MOTION_TASK_PRIORITY,
        nullptr);
}

void MotionCommandTask::taskEntry(void* task_context)
{
    static_cast<MotionCommandTask*>(task_context)->run();
}

void MotionCommandTask::run()
{
    model::MotionCommand command{};

    while (true) {
        if (xQueueReceive(motion_queue_, &command, portMAX_DELAY) == pdTRUE) {
            submitWhenReady(command);
        }
    }
}

void MotionCommandTask::submitWhenReady(const model::MotionCommand& command)
{
    while (!context_.submitCommand(command)) {
        vTaskDelay(pdMS_TO_TICKS(config::rtos::MOTION_TASK_IDLE_DELAY_MS));
    }
}

}  // namespace minicar::logic::motion_control
