#include <cstdint>
#include <cstdio>

#include "application/Mission.hpp"
#include "config/FreeRTOSConfig.hpp"
#include "driver/communication/SerialDriver.hpp"
#include "driver/imu/IMUDriver.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "driver/telemetry/ConsoleDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logic/SerialTask.hpp"
#include "logic/TelemetryTask.hpp"
#include "logic/motion_control/MotionCommandTask.hpp"
#include "logic/motion_control/MotionControlContext.hpp"
#include "logic/motion_control/MotionLoopTask.hpp"
#include "model/MotionCommand.hpp"
#include "model/TelemetryMessage.hpp"
#include "service/communication/CommunicationService.hpp"
#include "service/heading/HeadingService.hpp"
#include "service/motor/MotorControlService.hpp"
#include "service/odometry/OdometryService.hpp"
#include "service/telemetry/TelemetryService.hpp"

namespace {

constexpr const char* TAG = "MiniCar";

void publishStartupTelemetry(QueueHandle_t telemetry_queue, const char* text)
{
    minicar::model::TelemetryMessage message{};
    message.level = minicar::model::LogLevel::INFO;
    message.timestamp_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    std::snprintf(message.tag, sizeof(message.tag), "%s", "startup");
    std::snprintf(message.message, sizeof(message.message), "%s", text);
    (void)xQueueSend(telemetry_queue, &message, 0);
}

void logInitWarning(const char* name, esp_err_t result)
{
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "%s init pending: %s", name, esp_err_to_name(result));
    }
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    static driver::MotorEncoderDriver motor_driver;
    static driver::IMUDriver imu_driver;
    static driver::ConsoleDriver console_driver;
    static driver::SerialDriver serial_driver;

    static service::MotorControlService motor_service(motor_driver);
    static service::HeadingService heading_service(imu_driver);
    static service::OdometryService odometry_service;
    static service::TelemetryService telemetry_service(console_driver);
    static service::CommunicationService communication_service(serial_driver);

    static logic::motion_control::MotionControlContext motion_context;

    static QueueHandle_t motion_queue = xQueueCreate(
        config::rtos::MOTION_QUEUE_LENGTH,
        sizeof(model::MotionCommand));
    static QueueHandle_t telemetry_queue = xQueueCreate(
        config::rtos::TELEMETRY_QUEUE_LENGTH,
        sizeof(model::TelemetryMessage));

    if (motion_queue == nullptr || telemetry_queue == nullptr) {
        ESP_LOGE(TAG, "queue creation failed");
        return;
    }

    logInitWarning("telemetry", telemetry_service.init());
    logInitWarning("communication", communication_service.init());
    logInitWarning("motor", motor_service.init());
    logInitWarning("heading", heading_service.init());
    logInitWarning("odometry", odometry_service.init());

    static logic::motion_control::MotionCommandTask motion_command_task(motion_queue, motion_context);
    static logic::motion_control::MotionLoopTask motion_loop_task(
        motion_context,
        motor_service,
        heading_service,
        odometry_service);
    static logic::SerialTask serial_task(motion_queue, communication_service);
    static logic::TelemetryTask telemetry_task(telemetry_queue, telemetry_service);

    (void)telemetry_task.start();
    (void)motion_loop_task.start();
    (void)motion_command_task.start();
    (void)serial_task.start();

    publishStartupTelemetry(telemetry_queue, "mini car tasks started");
    (void)application::Mission::enqueueDemoMission(motion_queue);
}
