#include "driver/motor/MotorEncoderDriver.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "MotorDriverTest";
constexpr float TEST_OUTPUT = 0.25f;
constexpr TickType_t TEST_DURATION = pdMS_TO_TICKS(2000);
constexpr TickType_t STOP_DURATION = pdMS_TO_TICKS(1000);

void printFeedback(const minicar::driver::MotorEncoderDriver& motor)
{
    ESP_LOGI(
        TAG,
        "feedback: left_ticks=%ld right_ticks=%ld left_rpm=%.2f right_rpm=%.2f",
        static_cast<long>(motor.getLeftTicks()),
        static_cast<long>(motor.getRightTicks()),
        static_cast<double>(motor.getLeftRPM()),
        static_cast<double>(motor.getRightRPM()));
}

void runOutputTest(
    minicar::driver::MotorEncoderDriver& motor,
    const char* test_name,
    float left_output,
    float right_output)
{
    ESP_LOGI(
        TAG,
        "%s: expected output left=%.2f right=%.2f for 2 seconds",
        test_name,
        static_cast<double>(left_output),
        static_cast<double>(right_output));

    motor.setLeftOutput(left_output);
    motor.setRightOutput(right_output);
    vTaskDelay(TEST_DURATION);
    printFeedback(motor);

    motor.stop();
    ESP_LOGI(TAG, "%s: STOP for 1 second", test_name);
    vTaskDelay(STOP_DURATION);
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGW(TAG, "BENCH TEST: angkat roda dari lantai dan siapkan emergency stop");

    static minicar::driver::MotorEncoderDriver motor;
    const esp_err_t result = motor.init();
    if (result != ESP_OK) {
        motor.stop();
        ESP_LOGE(TAG, "motor init failed: %s", esp_err_to_name(result));
        return;
    }

    // Expected physical convention:
    // positive output = wheel rotates forward, negative output = reverse.
    runOutputTest(motor, "LEFT FORWARD", TEST_OUTPUT, 0.0f);
    runOutputTest(motor, "RIGHT FORWARD", 0.0f, TEST_OUTPUT);
    runOutputTest(motor, "BOTH FORWARD", TEST_OUTPUT, TEST_OUTPUT);
    runOutputTest(motor, "BOTH REVERSE", -TEST_OUTPUT, -TEST_OUTPUT);

    motor.stop();
    printFeedback(motor);
    ESP_LOGI(TAG, "motor driver bench test finished; outputs remain stopped");
}
