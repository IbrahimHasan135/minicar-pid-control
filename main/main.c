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
constexpr float CONTROL_DT_S = 0.05f;
constexpr float LOW_TEST_SPEED_MPS = 0.12f;

void runPhase(
    minicar::service::MotorControlService& motor_service,
    const char* phase,
    float left_target_mps,
    float right_target_mps,
    int duration_ms)
{
    ESP_LOGI(TAG, "PHASE=%s target_left=%.3f target_right=%.3f",
        phase,
        static_cast<double>(left_target_mps),
        static_cast<double>(right_target_mps));

    motor_service.resetControllers();
    motor_service.setVelocityTargets(left_target_mps, right_target_mps);

    const int steps = duration_ms / static_cast<int>(CONTROL_DT_S * 1000.0f);
    for (int i = 0; i < steps; ++i) {
        motor_service.refreshFeedback();
        motor_service.applyControl(CONTROL_DT_S);

        if ((i % 5) == 0) {
            ESP_LOGI(
                TAG,
                "phase=%s left_ticks=%ld right_ticks=%ld left_mps=%.3f right_mps=%.3f",
                phase,
                static_cast<long>(motor_service.getLeftTicks()),
                static_cast<long>(motor_service.getRightTicks()),
                static_cast<double>(motor_service.getLeftVelocityMps()),
                static_cast<double>(motor_service.getRightVelocityMps()));
        }

        vTaskDelay(CONTROL_PERIOD);
    }

    motor_service.stopMotor();
    motor_service.refreshFeedback();
    ESP_LOGI(
        TAG,
        "PHASE=%s DONE left_ticks=%ld right_ticks=%ld left_mps=%.3f right_mps=%.3f",
        phase,
        static_cast<long>(motor_service.getLeftTicks()),
        static_cast<long>(motor_service.getRightTicks()),
        static_cast<double>(motor_service.getLeftVelocityMps()),
        static_cast<double>(motor_service.getRightVelocityMps()));
}

}  // namespace

extern "C" void app_main(void)
{
    using namespace minicar;

    ESP_LOGI(TAG, "SERVICE MOTOR BENCH TEST: roda wajib terangkat dan emergency power-off siap");
    ESP_LOGI(TAG, "Default aktif hanya LEFT_MOTOR. Cek pin di MotorHardwareConfig.hpp sebelum flash");

    static driver::MotorEncoderDriver left_motor(config::motor_hardware::LEFT_MOTOR);
    static driver::MotorEncoderDriver right_motor(config::motor_hardware::RIGHT_MOTOR);
    static service::MotorControlService motor_service(left_motor, right_motor);

    const esp_err_t result = motor_service.init();
    if (result != ESP_OK) {
        motor_service.stopMotor();
        ESP_LOGE(TAG, "motor service init failed: %s", esp_err_to_name(result));
        return;
    }

    ESP_LOGI(TAG, "Boot safe stop 2s: motor harus diam");
    motor_service.stopMotor();
    vTaskDelay(pdMS_TO_TICKS(2000));

    runPhase(motor_service, "left_forward_low", LOW_TEST_SPEED_MPS, 0.0f, 3000);
    vTaskDelay(pdMS_TO_TICKS(1500));
    runPhase(motor_service, "left_reverse_low", -LOW_TEST_SPEED_MPS, 0.0f, 3000);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Bench sequence complete. Motor stopped; logs above menjadi bukti arah, ticks, dan feedback");
    motor_service.stopMotor();
}
