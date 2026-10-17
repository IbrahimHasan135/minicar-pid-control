#include "config/MotorHardwareConfig.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "SingleBtsTest";
constexpr TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(100);
constexpr int SAFE_START_DELAY_MS = 5000;
constexpr int FULL_POWER_DURATION_MS = 2000;
constexpr int STOP_DURATION_MS = 3000;

esp_err_t runPhase(
    minicar::driver::MotorEncoderDriver& motor,
    const char* phase,
    float output)
{
    ESP_LOGI(TAG, "PHASE=%s command=%.2f duration_ms=%d",
        phase,
        static_cast<double>(output),
        FULL_POWER_DURATION_MS);

    const esp_err_t result = motor.setOutput(output);
    if (result != ESP_OK) {
        motor.stop();
        ESP_LOGE(TAG, "PHASE=%s failed: %s", phase, esp_err_to_name(result));
        return result;
    }

    const int samples = FULL_POWER_DURATION_MS / static_cast<int>(SAMPLE_PERIOD * portTICK_PERIOD_MS);
    for (int i = 0; i < samples; ++i) {
        ESP_LOGI(TAG, "phase=%s ticks=%ld rpm=%.2f",
            phase,
            static_cast<long>(motor.getTicks()),
            static_cast<double>(motor.getRPM()));
        vTaskDelay(SAMPLE_PERIOD);
    }

    motor.stop();
    ESP_LOGI(TAG, "PHASE=%s DONE; motor STOP", phase);
    vTaskDelay(pdMS_TO_TICKS(STOP_DURATION_MS));
    return ESP_OK;
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    ESP_LOGW(TAG, "SINGLE BTS7960 DIAGNOSTIC: hanya motor kanan yang diuji");
    ESP_LOGW(TAG, "RPWM=GPIO22, LPWM=GPIO23, R_EN/L_EN=5V; GPIO25/26 TIDAK diinisialisasi");
    ESP_LOGW(TAG, "Lepas GY-511 karena GPIO22 juga dialokasikan sebagai SCL IMU");
    ESP_LOGW(TAG, "Roda wajib terangkat dan emergency power-off harus siap");

    static driver::MotorEncoderDriver motor(config::motor_hardware::RIGHT_MOTOR);
    const esp_err_t init_result = motor.init();
    if (init_result != ESP_OK) {
        motor.stop();
        ESP_LOGE(TAG, "motor init failed: %s", esp_err_to_name(init_result));
        return;
    }

    motor.stop();
    ESP_LOGI(TAG, "Boot safe stop %d ms", SAFE_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(SAFE_START_DELAY_MS));

    unsigned int cycle = 1;
    while (true) {
        ESP_LOGI(TAG, "SINGLE-MOTOR TEST CYCLE=%u START", cycle);

        // Sama dengan sketch Arduino: RPWM aktif, LPWM nol.
        if (runPhase(motor, "rpwm_gpio22_100pct", 1.0f) != ESP_OK) {
            break;
        }

        // Sama dengan sketch Arduino: LPWM aktif, RPWM nol.
        if (runPhase(motor, "lpwm_gpio23_100pct", -1.0f) != ESP_OK) {
            break;
        }

        ESP_LOGI(TAG, "SINGLE-MOTOR TEST CYCLE=%u DONE", cycle);
        ++cycle;
    }

    motor.stop();
    while (true) {
        ESP_LOGE(TAG, "TEST ABORTED; motor tetap STOP");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
