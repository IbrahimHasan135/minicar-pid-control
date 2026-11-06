#include "application/Mission.hpp"

#include "model/MotionCommand.hpp"

namespace minicar::application {

bool Mission::enqueueInitialTestMission(QueueHandle_t motion_queue)
{
    constexpr float TEST_DISTANCE_M = 1.0f;
    constexpr float TEST_SPEED_MPS = 0.2f;
    constexpr float RIGHT_TURN_DEG = 90.0f;

    return pushMove(motion_queue, TEST_DISTANCE_M, TEST_SPEED_MPS) &&
           pushTurn(motion_queue, RIGHT_TURN_DEG) &&
           pushStop(motion_queue);
}

bool Mission::enqueueDemoMission(QueueHandle_t motion_queue)
{
    return pushMove(motion_queue, 1.0f, 1.0f) &&
           pushTurn(motion_queue, 90.0f) &&
           pushMove(motion_queue, 2.0f, 0.5f) &&
           pushStop(motion_queue);
}

bool Mission::pushEncoderDemoMission(QueueHandle_t motion_queue)
{
    return pushMove(motion_queue, 2.0f, 0.2f) &&
           pushMove(motion_queue, 2.0f, 0.4f) &&
           pushMove(motion_queue, 2.0f, 0.58f) &&
           pushStop(motion_queue);
}

bool Mission::enqueueTurnTestMission(QueueHandle_t motion_queue)
{
    return pushTurn(motion_queue, 90.0f) &&
           pushWait(motion_queue, 5.0f) &&
           pushTurn(motion_queue, 90.0f) &&
           pushWait(motion_queue, 5.0f) &&
           pushTurn(motion_queue, 90.0f) &&
           pushWait(motion_queue, 5.0f) &&
           pushTurn(motion_queue, 90.0f) &&
           pushWait(motion_queue, 5.0f) &&
           pushStop(motion_queue);
}

bool Mission::enqueueSquareMission(QueueHandle_t motion_queue)
{
    constexpr float SIDE_DISTANCE_M = 1.0f;
    constexpr float FORWARD_SPEED_MPS = 0.4f;
    constexpr float RIGHT_TURN_DEG = 90.0f;

    return pushMove(motion_queue, SIDE_DISTANCE_M, FORWARD_SPEED_MPS) &&
           pushTurn(motion_queue, RIGHT_TURN_DEG) &&
           pushMove(motion_queue, SIDE_DISTANCE_M, FORWARD_SPEED_MPS) &&
           pushTurn(motion_queue, RIGHT_TURN_DEG) &&
           pushMove(motion_queue, SIDE_DISTANCE_M, FORWARD_SPEED_MPS) &&
           pushTurn(motion_queue, RIGHT_TURN_DEG) &&
           pushMove(motion_queue, SIDE_DISTANCE_M, FORWARD_SPEED_MPS) &&
           pushTurn(motion_queue, RIGHT_TURN_DEG) &&
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

bool Mission::pushWait(QueueHandle_t motion_queue, float duration_s)
{
    const model::MotionCommand command{model::MotionType::WAIT_SECONDS, duration_s, 0.0f};
    return xQueueSend(motion_queue, &command, 0) == pdTRUE;
}

bool Mission::pushStop(QueueHandle_t motion_queue)
{
    const model::MotionCommand command{model::MotionType::STOP, 0.0f, 0.0f};
    return xQueueSend(motion_queue, &command, 0) == pdTRUE;
}

}  // namespace minicar::application
