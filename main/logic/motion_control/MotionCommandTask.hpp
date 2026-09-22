#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logic/motion_control/MotionControlContext.hpp"
#include "model/MotionCommand.hpp"

namespace minicar::logic::motion_control {

class MotionCommandTask {
public:
    MotionCommandTask(QueueHandle_t motion_queue, MotionControlContext& context);

    BaseType_t start();

private:
    static void taskEntry(void* task_context);
    void run();
    void submitWhenReady(const model::MotionCommand& command);

    QueueHandle_t motion_queue_{nullptr};
    MotionControlContext& context_;
};

}  // namespace minicar::logic::motion_control
