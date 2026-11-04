#include <cstdint>
#include <cstdio>

#include "application/Mission.hpp"
#include "config/FreeRTOSConfig.hpp"
#include "config/MotorHardwareConfig.hpp"
#include "driver/imu/IMUDriver.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "driver/navigation/NavigationSerialDriver.hpp"
#include "driver/telemetry/ConsoleDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logic/TelemetryTask.hpp"
#include "logic/NavigationSerialTask.hpp"
#include "logic/motion_control/MotionCommandTask.hpp"
#include "logic/motion_control/MotionControlContext.hpp"
#include "logic/motion_control/MotionLoopTask.hpp"
#include "model/MotionCommand.hpp"
#include "model/TelemetryMessage.hpp"
#include "service/heading/HeadingService.hpp"
#include "service/motor/MotorControlService.hpp"
#include "service/navigation/NavigationSerialService.hpp"
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

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    static driver::MotorEncoderDriver left_motor_driver(config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor_driver(config::motor_hardware::RIGHT_MOTOR);
    static driver::IMUDriver imu_driver;
    static driver::ConsoleDriver console_driver;
    static driver::NavigationSerialDriver navigation_serial_driver;

    static service::MotorControlService motor_service(left_motor_driver, right_motor_driver);
    static service::HeadingService heading_service(imu_driver);
    static service::OdometryService odometry_service;
    static service::TelemetryService telemetry_service(console_driver);
    static service::NavigationSerialService navigation_serial_service(
        navigation_serial_driver);

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

    if (telemetry_service.init() != ESP_OK || motor_service.init() != ESP_OK ||
        odometry_service.init() != ESP_OK || heading_service.init() != ESP_OK ||
        navigation_serial_service.init() != ESP_OK) {
        ESP_LOGE(TAG, "required service initialization failed");
        motor_service.stopMotor();
        return;
    }

    ESP_LOGW(TAG, "IMU calibration sweep disabled; using raw magnetometer heading");

    // SerialTask/CommunicationService are also disabled until SerialDriver is
    // implemented. Telemetry still uses the ConsoleDriver and its queue.
    ESP_LOGI(TAG, "serial task/service disabled until SerialDriver is implemented");

    static logic::motion_control::MotionCommandTask motion_command_task(motion_queue, motion_context);
    static logic::motion_control::MotionLoopTask motion_loop_task(
        motion_context, motor_service, heading_service, odometry_service, telemetry_queue);
    static logic::TelemetryTask telemetry_task(telemetry_queue, telemetry_service);
    static logic::NavigationSerialTask navigation_serial_task(
        navigation_serial_service);

    if (navigation_serial_task.init() != ESP_OK) {
        ESP_LOGE(TAG, "navigation StreamBuffer creation failed");
        motor_service.stopMotor();
        return;
    }

    (void)telemetry_task.start();
    (void)motion_loop_task.start();
    (void)motion_command_task.start();
    if (navigation_serial_task.start() != ESP_OK) {
        ESP_LOGE(TAG, "navigation FreeRTOS tasks failed to start");
        motor_service.stopMotor();
        return;
    }
    // SerialTask is intentionally disabled until SerialDriver is implemented.

    publishStartupTelemetry(
        telemetry_queue,
        "integration mode ready; motor remains stopped; UART2 RX GPIO32");
    motor_service.stopMotor();
}
