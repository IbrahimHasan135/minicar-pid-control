#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "logic/motion_control/MotionControlContext.hpp"
#include "model/MotionCommand.hpp"
#include "model/TelemetryMessage.hpp"
#include "service/control/PIDController.hpp"
#include "service/heading/HeadingService.hpp"
#include "service/motor/MotorControlService.hpp"
#include "service/odometry/OdometryService.hpp"

namespace minicar::logic::motion_control {

class MotionLoopTask {
public:
    MotionLoopTask(
        MotionControlContext& context,
        service::MotorControlService& motor_service,
        service::HeadingService& heading_service,
        service::OdometryService& odometry_service,
        QueueHandle_t telemetry_queue);

    BaseType_t start();

private:
    static void taskEntry(void* task_context);
    void run();
    void step(float dt_s);

    void updateServices(float dt_s);
    void startPendingCommand();
    void startMove(const model::MotionCommand& command);
    void startTurn(const model::MotionCommand& command);
    void handleSnapshot(const MotionControlSnapshot& snapshot, float dt_s);
    void handleMove(const MotionControlSnapshot& snapshot, float dt_s);
    void handleTurn(const MotionControlSnapshot& snapshot, float dt_s);
    void stopAndIdle();
    void failSafeStop();
    void applySpeedControl(float dt_s);
    void publishTelemetry();

    bool servicesReady() const;
    bool isValidMove(const model::MotionCommand& command) const;
    bool isValidTurn(const model::MotionCommand& command) const;

    MotionControlContext& context_;
    service::MotorControlService& motor_service_;
    service::HeadingService& heading_service_;
    service::OdometryService& odometry_service_;
    QueueHandle_t telemetry_queue_{nullptr};
    service::PIDController left_speed_pid_;
    service::PIDController right_speed_pid_;
    float left_target_mps_{0.0f};
    float right_target_mps_{0.0f};
    float filtered_left_speed_mps_{0.0f};
    float filtered_right_speed_mps_{0.0f};
    float previous_left_output_{0.0f};
    float previous_right_output_{0.0f};
    bool feedback_filter_initialized_{false};
    TickType_t last_telemetry_tick_{0};
};

}  // namespace minicar::logic::motion_control
