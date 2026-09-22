#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "model/MotionCommand.hpp"
#include "service/communication/CommunicationService.hpp"

namespace minicar::logic {

class SerialTask {
public:
    SerialTask(QueueHandle_t motion_queue, service::CommunicationService& communication_service);

    BaseType_t start();

private:
    static void taskEntry(void* context);
    void run();

    QueueHandle_t motion_queue_{nullptr};
    service::CommunicationService& communication_service_;
};

}  // namespace minicar::logic
