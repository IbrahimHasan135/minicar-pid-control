#include "driver/imu/IMUDriver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <limits>

namespace {

constexpr const char* TAG = "IMU_BENCH";
constexpr TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(200);
constexpr TickType_t CALIBRATION_SAMPLE_PERIOD = pdMS_TO_TICKS(50);
constexpr uint32_t CALIBRATION_DURATION_MS = 30000;

}  // namespace

extern "C" void app_main(void)
{
    static minicar::driver::IMUDriver imu;

    ESP_LOGI(TAG, "GY-511/LSM303DLHC bench: SDA=GPIO21 SCL=GPIO22");
    ESP_LOGI(TAG, "Expected I2C addresses: accelerometer=0x19 magnetometer=0x1E");

    const esp_err_t init_result = imu.init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(init_result));
        return;
    }

    ESP_LOGI(TAG, "IMU initialized; starting sensor reads");
    ESP_LOGW(TAG, "Calibration starts: rotate the sensor/car horizontally through 360 degrees for 30 seconds");

    minicar::driver::MagnetometerCalibration calibration{
        {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()},
        {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()},
    };

    const TickType_t calibration_start = xTaskGetTickCount();
    const TickType_t calibration_duration = pdMS_TO_TICKS(CALIBRATION_DURATION_MS);
    TickType_t last_report = calibration_start;
    while ((xTaskGetTickCount() - calibration_start) < calibration_duration && imu.isHealthy()) {
        minicar::driver::MagneticFieldUt magnetic_field{};
        const esp_err_t result = imu.readMagneticField(magnetic_field);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed: %s", esp_err_to_name(result));
            return;
        }

        calibration.minimum.x = std::min(calibration.minimum.x, magnetic_field.x);
        calibration.minimum.y = std::min(calibration.minimum.y, magnetic_field.y);
        calibration.minimum.z = std::min(calibration.minimum.z, magnetic_field.z);
        calibration.maximum.x = std::max(calibration.maximum.x, magnetic_field.x);
        calibration.maximum.y = std::max(calibration.maximum.y, magnetic_field.y);
        calibration.maximum.z = std::max(calibration.maximum.z, magnetic_field.z);

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_report) >= pdMS_TO_TICKS(1000)) {
            const uint32_t elapsed_ms = (now - calibration_start) * portTICK_PERIOD_MS;
            ESP_LOGI(TAG, "Calibrating... %lu/%lu seconds", elapsed_ms / 1000, CALIBRATION_DURATION_MS / 1000);
            last_report = now;
        }
        vTaskDelay(CALIBRATION_SAMPLE_PERIOD);
    }

    ESP_LOGI(
        TAG,
        "Calibration values: min={%+.2f,%+.2f,%+.2f} max={%+.2f,%+.2f,%+.2f} uT",
        calibration.minimum.x,
        calibration.minimum.y,
        calibration.minimum.z,
        calibration.maximum.x,
        calibration.maximum.y,
        calibration.maximum.z);

    const esp_err_t calibration_result = imu.setMagnetometerCalibration(calibration);
    if (calibration_result != ESP_OK) {
        ESP_LOGE(TAG, "Calibration invalid: rotate the sensor farther through a full horizontal circle");
        return;
    }

    ESP_LOGI(TAG, "Calibration applied; heading output started");
    while (imu.isHealthy()) {
        minicar::driver::AccelerationMps2 acceleration{};
        minicar::driver::MagneticFieldUt magnetic_field{};
        float heading_deg = 0.0f;

        const esp_err_t acceleration_result = imu.readAcceleration(acceleration);
        const esp_err_t magnetic_result = imu.readMagneticField(magnetic_field);
        const esp_err_t heading_result = imu.readHeadingDeg(heading_deg);
        if (acceleration_result != ESP_OK || magnetic_result != ESP_OK || heading_result != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Sensor read failed: accel=%s mag=%s heading=%s",
                esp_err_to_name(acceleration_result),
                esp_err_to_name(magnetic_result),
                esp_err_to_name(heading_result));
            break;
        }

        ESP_LOGI(
            TAG,
            "heading=%6.1f deg | accel[m/s2] x=%+.3f y=%+.3f z=%+.3f | mag[uT] x=%+.2f y=%+.2f z=%+.2f",
            heading_deg,
            acceleration.x,
            acceleration.y,
            acceleration.z,
            magnetic_field.x,
            magnetic_field.y,
            magnetic_field.z);

        vTaskDelay(SAMPLE_PERIOD);
    }

    ESP_LOGE(TAG, "IMU bench stopped because driver is unhealthy");
}
