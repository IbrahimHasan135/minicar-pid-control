#include <cstdint>
#include <cstdio>

#include "config/FreeRTOSConfig.hpp"
#include "config/MotorHardwareConfig.hpp"
#include "config/NetworkConfig.hpp"
#include "config/RobotConfig.hpp"
#include "driver/communication/MqttDriver.hpp"
#include "driver/communication/ProvisioningPortalDriver.hpp"
#include "driver/communication/WifiDriver.hpp"
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
#include "logic/communication/MqttPublishEventQueue.hpp"
#include "logic/communication/WifiMqttTask.hpp"
#include "model/TelemetryMessage.hpp"
#include "service/heading/HeadingService.hpp"
#include "service/communication/MqttCommunicationService.hpp"
#include "service/communication/WifiService.hpp"
#include "service/configuration/NetworkSettingsService.hpp"
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

    static service::NetworkSettingsService network_settings_service;
    if (network_settings_service.init() != ESP_OK) {
        ESP_LOGE(TAG, "network settings initialization failed");
        return;
    }
    const model::NetworkSettings& network_settings = network_settings_service.settings();

    static driver::MotorEncoderDriver left_motor_driver(config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor_driver(config::motor_hardware::RIGHT_MOTOR);
    static driver::IMUDriver imu_driver;
    static driver::ConsoleDriver console_driver;
    static driver::NavigationSerialDriver navigation_serial_driver;
    static driver::WifiDriver wifi_driver;
    static driver::MqttDriver mqtt_driver;
    static driver::ProvisioningPortalDriver provisioning_portal_driver;

    static service::MotorControlService motor_service(left_motor_driver, right_motor_driver);
    static service::HeadingService heading_service(imu_driver);
    static service::OdometryService odometry_service;
    static service::TelemetryService telemetry_service(console_driver);
    static service::NavigationSerialService navigation_serial_service(
        navigation_serial_driver);
    static service::WifiService wifi_service(
        wifi_driver, network_settings.wifi_ssid, network_settings.wifi_password,
        config::network::PROVISIONING_AP_SSID,
        config::network::PROVISIONING_AP_PASSWORD);
    static service::MqttCommunicationService mqtt_communication_service(
        mqtt_driver, network_settings.mqtt_uri,
        network_settings.mqtt_username, network_settings.mqtt_password,
        network_settings.mqtt_client_id, config::network::MQTT_PUBLISH_QOS);

    static QueueHandle_t telemetry_queue = xQueueCreate(
        config::rtos::TELEMETRY_QUEUE_LENGTH,
        sizeof(model::TelemetryMessage));
    static logic::MqttPublishEventQueue mqtt_publish_queue;

    if (telemetry_queue == nullptr) {
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

    if (wifi_service.init() != ESP_OK ||
        provisioning_portal_driver.start(network_settings_service) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi provisioning portal initialization failed");
        motor_service.stopMotor();
        return;
    }

    // SerialTask/CommunicationService are also disabled until SerialDriver is
    // implemented. Telemetry still uses the ConsoleDriver and its queue.
    ESP_LOGI(TAG, "serial task/service disabled until SerialDriver is implemented");

    static logic::TelemetryTask telemetry_task(telemetry_queue, telemetry_service);
    static logic::NavigationSerialTask navigation_serial_task(
        navigation_serial_service, mqtt_publish_queue);
    if (navigation_serial_task.init() != ESP_OK) {
        ESP_LOGE(TAG, "navigation StreamBuffer creation failed");
        motor_service.stopMotor();
        return;
    }
    static logic::WifiMqttTask wifi_mqtt_task(
        mqtt_publish_queue, wifi_service, mqtt_communication_service);

    // Static navigation-data acquisition mode: initialize the motor hardware
    // into a known safe state, but do not start any motion-control task or
    // enqueue any movement mission.
    motor_service.stopMotor();
    (void)telemetry_task.start();
    const esp_err_t mqtt_queue_result = mqtt_publish_queue.create(
        config::robot::IMU_DIAGNOSTIC_MQTT_QUEUE_LENGTH);
    if (mqtt_queue_result == ESP_OK) {
        const BaseType_t task_result = wifi_mqtt_task.start();
        if (task_result != pdPASS) ESP_LOGE(TAG, "Wi-Fi/MQTT task creation failed");
    } else {
        ESP_LOGE(TAG, "MQTT publish queue unavailable; motion tasks continue without MQTT");
    }

    if (navigation_serial_task.start() != ESP_OK) {
        ESP_LOGE(TAG, "navigation FreeRTOS tasks failed to start");
        motor_service.stopMotor();
        return;
    }
    // SerialTask is intentionally disabled until SerialDriver is implemented.

    publishStartupTelemetry(
        telemetry_queue,
        "static serial acquisition mode; motor stopped; UART-to-MQTT active");
}
