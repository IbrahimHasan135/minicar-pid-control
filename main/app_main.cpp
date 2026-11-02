#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "application/Mission.hpp"
#include "config/FreeRTOSConfig.hpp"
#include "config/MotorHardwareConfig.hpp"
#include "driver/imu/IMUDriver.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "driver/telemetry/ConsoleDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "logic/TelemetryTask.hpp"
#include "logic/motion_control/MotionCommandTask.hpp"
#include "logic/motion_control/MotionControlContext.hpp"
#include "logic/motion_control/MotionLoopTask.hpp"
#include "model/MotionCommand.hpp"
#include "model/TelemetryMessage.hpp"
#include "service/heading/HeadingService.hpp"
#include "service/motor/MotorControlService.hpp"
#include "service/odometry/OdometryService.hpp"
#include "service/telemetry/TelemetryService.hpp"

namespace {

constexpr const char* TAG = "MiniCar";
constexpr TickType_t IMU_CALIBRATION_SAMPLE_PERIOD = pdMS_TO_TICKS(50);
constexpr uint32_t IMU_CALIBRATION_DURATION_MS = 30000;

void publishStartupTelemetry(QueueHandle_t telemetry_queue, const char* text)
{
    minicar::model::TelemetryMessage message{};
    message.level = minicar::model::LogLevel::INFO;
    message.timestamp_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    std::snprintf(message.tag, sizeof(message.tag), "%s", "startup");
    std::snprintf(message.message, sizeof(message.message), "%s", text);
    (void)xQueueSend(telemetry_queue, &message, 0);
}

esp_err_t calibrateImu(minicar::driver::IMUDriver& imu)
{
    esp_err_t result = imu.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(result));
        return result;
    }

    ESP_LOGW(
        TAG,
        "IMU calibration: rotate the car horizontally through 360 degrees for 30 seconds");

    minicar::driver::MagnetometerCalibration calibration{
        {std::numeric_limits<float>::max(),
         std::numeric_limits<float>::max(),
         std::numeric_limits<float>::max()},
        {std::numeric_limits<float>::lowest(),
         std::numeric_limits<float>::lowest(),
         std::numeric_limits<float>::lowest()},
    };

    const TickType_t calibration_start = xTaskGetTickCount();
    const TickType_t calibration_duration = pdMS_TO_TICKS(IMU_CALIBRATION_DURATION_MS);
    TickType_t last_report = calibration_start;

    while ((xTaskGetTickCount() - calibration_start) < calibration_duration &&
           imu.isHealthy()) {
        minicar::driver::MagneticFieldUt magnetic_field{};
        result = imu.readMagneticField(magnetic_field);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "IMU calibration read failed: %s", esp_err_to_name(result));
            return result;
        }

        calibration.minimum.x = std::min(calibration.minimum.x, magnetic_field.x);
        calibration.minimum.y = std::min(calibration.minimum.y, magnetic_field.y);
        calibration.minimum.z = std::min(calibration.minimum.z, magnetic_field.z);
        calibration.maximum.x = std::max(calibration.maximum.x, magnetic_field.x);
        calibration.maximum.y = std::max(calibration.maximum.y, magnetic_field.y);
        calibration.maximum.z = std::max(calibration.maximum.z, magnetic_field.z);

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_report) >= pdMS_TO_TICKS(1000)) {
            const uint32_t elapsed_ms =
                static_cast<uint32_t>((now - calibration_start) * portTICK_PERIOD_MS);
            ESP_LOGI(
                TAG,
                "IMU calibrating: %lu/%lu seconds",
                static_cast<unsigned long>(elapsed_ms / 1000),
                static_cast<unsigned long>(IMU_CALIBRATION_DURATION_MS / 1000));
            last_report = now;
        }

        vTaskDelay(IMU_CALIBRATION_SAMPLE_PERIOD);
    }

    ESP_LOGI(
        TAG,
        "IMU calibration values: min={%+.2f,%+.2f,%+.2f} max={%+.2f,%+.2f,%+.2f} uT",
        static_cast<double>(calibration.minimum.x),
        static_cast<double>(calibration.minimum.y),
        static_cast<double>(calibration.minimum.z),
        static_cast<double>(calibration.maximum.x),
        static_cast<double>(calibration.maximum.y),
        static_cast<double>(calibration.maximum.z));

    result = imu.setMagnetometerCalibration(calibration);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "IMU calibration invalid: %s", esp_err_to_name(result));
        return result;
    }

    ESP_LOGI(TAG, "IMU calibration applied");
    return ESP_OK;
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    static driver::MotorEncoderDriver left_motor_driver(config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor_driver(config::motor_hardware::RIGHT_MOTOR);
    static driver::IMUDriver imu_driver;
    static driver::ConsoleDriver console_driver;

    static service::MotorControlService motor_service(left_motor_driver, right_motor_driver);
    static service::HeadingService heading_service(imu_driver);
    static service::OdometryService odometry_service;
    static service::TelemetryService telemetry_service(console_driver);

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
        odometry_service.init() != ESP_OK || calibrateImu(imu_driver) != ESP_OK ||
        heading_service.init() != ESP_OK) {
        ESP_LOGE(TAG, "required service initialization failed");
        motor_service.stopMotor();
        return;
    }

    // SerialTask/CommunicationService are also disabled until SerialDriver is
    // implemented. Telemetry still uses the ConsoleDriver and its queue.
    ESP_LOGI(TAG, "serial task/service disabled until SerialDriver is implemented");

    static logic::motion_control::MotionCommandTask motion_command_task(motion_queue, motion_context);
    static logic::motion_control::MotionLoopTask motion_loop_task(
        motion_context, motor_service, heading_service, odometry_service, telemetry_queue);
    static logic::TelemetryTask telemetry_task(telemetry_queue, telemetry_service);

    (void)telemetry_task.start();
    (void)motion_loop_task.start();
    (void)motion_command_task.start();
    // SerialTask is intentionally disabled until SerialDriver is implemented.

    publishStartupTelemetry(telemetry_queue, "mini car tasks started");
    (void)application::Mission::enqueueDemoMission(motion_queue);
}
