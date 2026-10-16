#include "config/MotorHardwareConfig.hpp"
#include "driver/motor/MotorEncoderDriver.hpp"
#include "service/motor/MotorControlService.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "MotorDriverTest";
constexpr TickType_t CONTROL_PERIOD = pdMS_TO_TICKS(50);
constexpr int SAFE_START_DELAY_MS = 5000;
constexpr int STOP_BETWEEN_PHASES_MS = 1500;
constexpr float OUTPUT_LOW = 0.20f;
constexpr float OUTPUT_MEDIUM = 0.35f;
constexpr float OUTPUT_HIGH = 0.50f;

void waitWithMotorStopped(minicar::service::MotorControlService& motor_service)
{
    motor_service.stopMotor();
    vTaskDelay(pdMS_TO_TICKS(STOP_BETWEEN_PHASES_MS));
}

void runOpenLoopPhase(
    minicar::service::MotorControlService& motor_service,
    const char* phase,
    float left_output,
    float right_output,
    int duration_ms)
{
    ESP_LOGI(TAG, "PHASE=%s left_output=%.2f right_output=%.2f duration_ms=%d",
        phase,
        static_cast<double>(left_output),
        static_cast<double>(right_output),
        duration_ms);

    motor_service.setOpenLoopOutputs(left_output, right_output);

    const int steps = duration_ms / static_cast<int>(CONTROL_PERIOD * portTICK_PERIOD_MS);
    for (int i = 0; i < steps; ++i) {
        motor_service.refreshFeedback();

        if ((i % 5) == 0) {
            ESP_LOGI(
                TAG,
                "phase=%s left_output=%.2f right_output=%.2f left_ticks=%ld right_ticks=%ld left_rpm=%.2f right_rpm=%.2f",
                phase,
                static_cast<double>(left_output),
                static_cast<double>(right_output),
                static_cast<long>(motor_service.getLeftTicks()),
                static_cast<long>(motor_service.getRightTicks()),
                static_cast<double>(motor_service.getLeftRPM()),
                static_cast<double>(motor_service.getRightRPM()));
        }

        vTaskDelay(CONTROL_PERIOD);
    }

    motor_service.stopMotor();
    motor_service.refreshFeedback();
    ESP_LOGI(
        TAG,
        "PHASE=%s DONE motor=STOP left_ticks=%ld right_ticks=%ld left_rpm=%.2f right_rpm=%.2f",
        phase,
        static_cast<long>(motor_service.getLeftTicks()),
        static_cast<long>(motor_service.getRightTicks()),
        static_cast<double>(motor_service.getLeftRPM()),
        static_cast<double>(motor_service.getRightRPM()));
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    ESP_LOGI(TAG, "OPEN-LOOP MOTOR BENCH TEST: roda wajib terangkat dan emergency power-off siap");
    ESP_LOGI(TAG, "PID TIDAK DIGUNAKAN. R_EN/L_EN kedua BTS harus terhubung langsung ke regulated 5 V");
    ESP_LOGI(TAG, "PWM kiri RPWM/LPWM=25/26; PWM kanan RPWM/LPWM=18/19; PWM=1 kHz 8-bit");

    static driver::MotorEncoderDriver left_motor(config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor(config::motor_hardware::RIGHT_MOTOR);
    static service::MotorControlService motor_service(left_motor, right_motor);

    const esp_err_t result = motor_service.init();
    if (result != ESP_OK) {
        motor_service.stopMotor();
        ESP_LOGE(TAG, "motor service init failed: %s", esp_err_to_name(result));
        return;
    }

    ESP_LOGI(TAG, "Boot safe stop %d ms: motor harus diam", SAFE_START_DELAY_MS);
    motor_service.stopMotor();
    vTaskDelay(pdMS_TO_TICKS(SAFE_START_DELAY_MS));

    runOpenLoopPhase(motor_service, "left_forward_20pct", OUTPUT_LOW, 0.0f, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "left_forward_35pct", OUTPUT_MEDIUM, 0.0f, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "left_forward_50pct", OUTPUT_HIGH, 0.0f, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "left_reverse_35pct", -OUTPUT_MEDIUM, 0.0f, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "right_forward_20pct", 0.0f, OUTPUT_LOW, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "right_forward_35pct", 0.0f, OUTPUT_MEDIUM, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "right_forward_50pct", 0.0f, OUTPUT_HIGH, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "right_reverse_35pct", 0.0f, -OUTPUT_MEDIUM, 1500);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "both_forward_35pct", OUTPUT_MEDIUM, OUTPUT_MEDIUM, 2000);
    waitWithMotorStopped(motor_service);
    runOpenLoopPhase(motor_service, "both_reverse_35pct", -OUTPUT_MEDIUM, -OUTPUT_MEDIUM, 2000);

    ESP_LOGI(TAG, "Open-loop bench sequence complete. Motor stopped; PID tidak digunakan");
    motor_service.stopMotor();

    while (true) {
        ESP_LOGI(TAG, "TEST COMPLETE: motor tetap STOP. Tekan EN/RESET untuk mengulang");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
