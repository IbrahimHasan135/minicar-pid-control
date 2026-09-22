#include "application/Mission.hpp"

#include "model/MotionCommand.hpp"

namespace minicar::application {

bool Mission::enqueueDemoMission(QueueHandle_t motion_queue)
{
    return pushMove(motion_queue, 1.0f, 1.0f) &&
           pushTurn(motion_queue, 90.0f) &&
           pushMove(motion_queue, 2.0f, 0.5f) &&
           pushStop(motion_queue);
}

bool Mission::pushMove(QueueHandle_t motion_queue, float distance_m, float speed_mps)
{
    const model::MotionCommand command{model::MotionType::MOVE_DISTANCE, distance_m, speed_mps};
    return xQueueSend(motion_queue, &command, 0) == pdTRUE;
}

bool Mission::pushTurn(QueueHandle_t motion_queue, float angle_deg)
{
    const model::MotionCommand command{model::MotionType::TURN_ANGLE, angle_deg, 0.0f};
    return xQueueSend(motion_queue, &command, 0) == pdTRUE;
}

bool Mission::pushStop(QueueHandle_t motion_queue)
{
    const model::MotionCommand command{model::MotionType::STOP, 0.0f, 0.0f};
    return xQueueSend(motion_queue, &command, 0) == pdTRUE;
}

}  // namespace minicar::application
