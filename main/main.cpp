#include <cstdint>
#include <algorithm>
#include <cmath>

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

struct MotorTestPhase {
    float target_speed_mps;
    uint32_t duration_ms;
};

constexpr MotorTestPhase MOTOR_TEST_PHASES[] = {
    {0.20f, 15000},
    {0.00f, 5000},
    {0.40f, 15000},
    {0.00f, 5000},
    {0.58f, 15000},
    {0.00f, 5000},
};

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
    static service::PIDController speed_sync_pid(
        config::pid::SPEED_SYNC_KP,
        config::pid::SPEED_SYNC_KI,
        config::pid::SPEED_SYNC_KD);

    left_speed_pid.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    right_speed_pid.setOutputLimit(config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
    left_speed_pid.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
    right_speed_pid.setIntegralLimit(-config::pid::SPEED_INTEGRAL_LIMIT, config::pid::SPEED_INTEGRAL_LIMIT);
    speed_sync_pid.setOutputLimit(
        -config::pid::SPEED_SYNC_OUTPUT_LIMIT,
        config::pid::SPEED_SYNC_OUTPUT_LIMIT);
    speed_sync_pid.setIntegralLimit(
        -config::pid::SPEED_SYNC_INTEGRAL_LIMIT,
        config::pid::SPEED_SYNC_INTEGRAL_LIMIT);

    ESP_LOGI(TAG, "starting motor and IMU driver development mode");

    const bool motor_ready = logInitResult("motor service", motor_service.init());
    const bool imu_ready = logInitResult("IMU", imu_driver.init());

    TickType_t last_phase_tick = xTaskGetTickCount();
    TickType_t last_control_tick = last_phase_tick;
    size_t phase_index = 0;
    float target_speed_mps = MOTOR_TEST_PHASES[phase_index].target_speed_mps;
    float filtered_left_speed_mps = 0.0f;
    float filtered_right_speed_mps = 0.0f;
    float previous_left_output = 0.0f;
    float previous_right_output = 0.0f;
    bool feedback_filter_initialized = false;

    while (true) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_phase_tick) >=
            pdMS_TO_TICKS(MOTOR_TEST_PHASES[phase_index].duration_ms)) {
            phase_index = (phase_index + 1) %
                          (sizeof(MOTOR_TEST_PHASES) / sizeof(MOTOR_TEST_PHASES[0]));
            target_speed_mps = MOTOR_TEST_PHASES[phase_index].target_speed_mps;
            left_speed_pid.reset();
            right_speed_pid.reset();
            speed_sync_pid.reset();
            filtered_left_speed_mps = 0.0f;
            filtered_right_speed_mps = 0.0f;
            previous_left_output = 0.0f;
            previous_right_output = 0.0f;
            feedback_filter_initialized = false;

            ESP_LOGI(TAG, "PID test phase=%u target=%.3f m/s duration=%lu ms",
                     static_cast<unsigned>(phase_index),
                     static_cast<double>(target_speed_mps),
                     static_cast<unsigned long>(MOTOR_TEST_PHASES[phase_index].duration_ms));
            last_phase_tick = now;
        }

        if (motor_ready) {
            motor_service.refreshFeedback();

            float clamped_left_output = 0.0f;
            float clamped_right_output = 0.0f;
            if (std::fabs(target_speed_mps) <= 0.001f) {
                // Do not use reverse PID braking at zero target; it causes oscillation
                // when the encoder reports residual motion with alternating signs.
                left_speed_pid.reset();
                right_speed_pid.reset();
                speed_sync_pid.reset();
                filtered_left_speed_mps = 0.0f;
                filtered_right_speed_mps = 0.0f;
                feedback_filter_initialized = false;
                previous_left_output = 0.0f;
                previous_right_output = 0.0f;
                motor_service.stopMotor();
            } else {
                const float left_speed_mps = motor_service.getLeftVelocityMps();
                const float right_speed_mps = motor_service.getRightVelocityMps();
                if (!feedback_filter_initialized) {
                    filtered_left_speed_mps = left_speed_mps;
                    filtered_right_speed_mps = right_speed_mps;
                    feedback_filter_initialized = true;
                } else {
                    constexpr float alpha = config::pid::SPEED_FEEDBACK_FILTER_ALPHA;
                    filtered_left_speed_mps += alpha * (left_speed_mps - filtered_left_speed_mps);
                    filtered_right_speed_mps += alpha * (right_speed_mps - filtered_right_speed_mps);
                }

                // A negative correction means the right wheel is slower: reduce
                // left output and add the same amount to the right output.
                const float sync_correction = speed_sync_pid.update(
                    0.0f,
                    filtered_left_speed_mps - filtered_right_speed_mps,
                    config::rtos::CONTROL_PERIOD_S);
                const float left_output = left_speed_pid.update(
                    target_speed_mps, filtered_left_speed_mps,
                    config::rtos::CONTROL_PERIOD_S) +
                    (config::pid::SPEED_FEED_FORWARD * target_speed_mps) +
                    sync_correction;
                const float right_output = right_speed_pid.update(
                    target_speed_mps, filtered_right_speed_mps,
                    config::rtos::CONTROL_PERIOD_S) +
                    (config::pid::SPEED_FEED_FORWARD * target_speed_mps) -
                    sync_correction;
                const float max_output_step =
                    config::pid::SPEED_OUTPUT_SLEW_RATE * config::rtos::CONTROL_PERIOD_S;
                clamped_left_output = std::clamp(
                    std::clamp(left_output, previous_left_output - max_output_step,
                               previous_left_output + max_output_step),
                    config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
                clamped_right_output = std::clamp(
                    std::clamp(right_output, previous_right_output - max_output_step,
                               previous_right_output + max_output_step),
                    config::robot::MIN_MOTOR_OUTPUT, config::robot::MAX_MOTOR_OUTPUT);
                previous_left_output = clamped_left_output;
                previous_right_output = clamped_right_output;
                motor_service.setMotorOutputs(clamped_left_output, clamped_right_output);
            }

            if ((now - last_control_tick) >= pdMS_TO_TICKS(DRIVER_MONITOR_PERIOD_MS)) {
                ESP_LOGI(
                    TAG,
                    "left motor: ticks=%ld rpm=%.3f speed=%.3f m/s output=%.3f",
                    static_cast<long>(motor_service.getLeftTicks()),
                    static_cast<double>(motor_service.getLeftRPM()),
                    static_cast<double>(motor_service.getLeftVelocityMps()),
                    static_cast<double>(clamped_left_output));
                ESP_LOGI(
                    TAG,
                    "right motor: ticks=%ld rpm=%.3f speed=%.3f m/s output=%.3f",
                    static_cast<long>(motor_service.getRightTicks()),
                    static_cast<double>(motor_service.getRightRPM()),
                    static_cast<double>(motor_service.getRightVelocityMps()),
                    static_cast<double>(clamped_right_output));
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
