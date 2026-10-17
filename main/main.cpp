#include <cstdint>

#include "driver/imu/IMUDriver.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "DriverDevelopment";
constexpr uint32_t DRIVER_MONITOR_PERIOD_MS = 500;

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

    // Driver-only bring-up entry point. Application, logic tasks, and services are
    // intentionally not instantiated while the hardware drivers are developed.
    static driver::MotorEncoderDriver left_motor_driver;
    static driver::MotorEncoderDriver right_motor_driver;
    static driver::IMUDriver imu_driver;

    ESP_LOGI(TAG, "starting motor and IMU driver development mode");

    const bool left_motor_ready = logInitResult("left motor", left_motor_driver.init());
    const bool right_motor_ready = logInitResult("right motor", right_motor_driver.init());
    const bool imu_ready = logInitResult("IMU", imu_driver.init());

    if (left_motor_ready) {
        left_motor_driver.stop();
    }
    if (right_motor_ready) {
        right_motor_driver.stop();
    }

    while (true) {
        if (left_motor_ready) {
            ESP_LOGI(
                TAG,
                "left motor: ticks=%ld rpm=%.3f",
                static_cast<long>(left_motor_driver.getTicks()),
                static_cast<double>(left_motor_driver.getRPM()));
        }

        if (right_motor_ready) {
            ESP_LOGI(
                TAG,
                "right motor: ticks=%ld rpm=%.3f",
                static_cast<long>(right_motor_driver.getTicks()),
                static_cast<double>(right_motor_driver.getRPM()));
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

        vTaskDelay(pdMS_TO_TICKS(DRIVER_MONITOR_PERIOD_MS));
    }
}
