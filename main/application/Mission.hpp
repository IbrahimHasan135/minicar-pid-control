#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace minicar::application {

class Mission {
public:
    static bool enqueueDemoMission(QueueHandle_t motion_queue);

private:
    static bool pushMove(QueueHandle_t motion_queue, float distance_m, float speed_mps);
    static bool pushTurn(QueueHandle_t motion_queue, float angle_deg);
    static bool pushStop(QueueHandle_t motion_queue);
};

}  // namespace minicar::application
