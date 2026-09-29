#include "driver/motor/MotorEncoderDriver.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "MotorDriverTest";
constexpr TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(250);

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ENCODER-ONLY TEST: motor output is not activated");
    ESP_LOGI(TAG, "Putar output shaft manual: maju harus +ticks, mundur harus -ticks");

    static minicar::driver::MotorEncoderDriver motor;
    const esp_err_t result = motor.init();
    if (result != ESP_OK) {
        motor.stop();
        ESP_LOGE(TAG, "motor init failed: %s", esp_err_to_name(result));
        return;
    }

    while (true) {
        ESP_LOGI(
            TAG,
            "ticks=%ld rpm=%.2f",
            static_cast<long>(motor.getTicks()),
            static_cast<double>(motor.getRPM()));
        vTaskDelay(SAMPLE_PERIOD);
    }
}
