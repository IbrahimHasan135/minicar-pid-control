#include <cstdint>

#include "config/MotorHardwareConfig.hpp"
#include "config/FreeRTOSConfig.hpp"
#include "config/PIDConfig.hpp"
#include "config/RobotConfig.hpp"
#include "driver/imu/IMUDriver.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "service/control/PIDController.hpp"
#include "service/motor/MotorControlService.hpp"

namespace {

constexpr const char* TAG = "DriverDevelopment";
constexpr uint32_t DRIVER_MONITOR_PERIOD_MS = 500;
constexpr uint32_t MOTOR_TEST_STEP_MS = 2000;

constexpr float MOTOR_TEST_TARGETS_MPS[] = {0.0f, 0.15f, 0.0f, -0.15f};

bool logInitResult(const char* driver_name, esp_err_t result)
{
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "%s init failed: %s", driver_name, esp_err_to_name(result));
        return false;
    }

    ESP_LOGI(TAG, "%s init OK", driver_name);
    return true;
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    static driver::MotorEncoderDriver left_motor_driver(
        config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor_driver(
        config::motor_hardware::RIGHT_MOTOR);
    static driver::IMUDriver imu_driver;
    static service::MotorControlService motor_service(
        left_motor_driver, right_motor_driver);
    static service::PIDController left_speed_pid(
        config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD);
    static service::PIDController right_speed_pid(
        config::pid::SPEED_KP, config::pid::SPEED_KI, config::pid::SPEED_KD);

    left_speed_pid.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    right_speed_pid.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    left_speed_pid.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
    right_speed_pid.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);

    ESP_LOGI(TAG, "starting motor and IMU driver development mode");

    const bool motor_ready = logInitResult("motor service", motor_service.init());
    const bool imu_ready = logInitResult("IMU", imu_driver.init());

    TickType_t last_motor_step = xTaskGetTickCount();
    TickType_t last_control_tick = last_motor_step;
    size_t motor_step = 0;
    float target_speed_mps = 0.0f;

    while (true) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_motor_step) >= pdMS_TO_TICKS(MOTOR_TEST_STEP_MS)) {
            motor_step = (motor_step + 1) %
                         (sizeof(MOTOR_TEST_TARGETS_MPS) / sizeof(MOTOR_TEST_TARGETS_MPS[0]));
            target_speed_mps = MOTOR_TEST_TARGETS_MPS[motor_step];
            left_speed_pid.reset();
            right_speed_pid.reset();

            ESP_LOGI(TAG, "dual motor PID target=%.3f m/s",
                     static_cast<double>(target_speed_mps));
            last_motor_step = now;
        }

        if (motor_ready) {
            motor_service.refreshFeedback();

            const float left_output = left_speed_pid.update(
                target_speed_mps, motor_service.getLeftVelocityMps(),
                config::rtos::CONTROL_PERIOD_S);
            const float right_output = right_speed_pid.update(
                target_speed_mps, motor_service.getRightVelocityMps(),
                config::rtos::CONTROL_PERIOD_S);
            motor_service.setMotorOutputs(left_output, right_output);

            if ((now - last_control_tick) >= pdMS_TO_TICKS(DRIVER_MONITOR_PERIOD_MS)) {
                ESP_LOGI(
                    TAG,
                    "left motor: ticks=%ld rpm=%.3f speed=%.3f m/s output=%.3f",
                    static_cast<long>(motor_service.getLeftTicks()),
                    static_cast<double>(motor_service.getLeftRPM()),
                    static_cast<double>(motor_service.getLeftVelocityMps()),
                    static_cast<double>(left_output));
                ESP_LOGI(
                    TAG,
                    "right motor: ticks=%ld rpm=%.3f speed=%.3f m/s output=%.3f",
                    static_cast<long>(motor_service.getRightTicks()),
                    static_cast<double>(motor_service.getRightRPM()),
                    static_cast<double>(motor_service.getRightVelocityMps()),
                    static_cast<double>(right_output));
                last_control_tick = now;
            }
        } else {
            motor_service.stopMotor();
        }

        if (imu_ready) {
            if (imu_driver.isHealthy()) {
                ESP_LOGI(
                    TAG,
                    "IMU: yaw_rate=%.3f dps",
                    static_cast<double>(imu_driver.getYawRateDps()));
            } else {
                ESP_LOGW(TAG, "IMU is not healthy");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(config::rtos::CONTROL_PERIOD_MS));
    }
}
