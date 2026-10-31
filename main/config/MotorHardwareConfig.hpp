#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

namespace minicar::config::motor_hardware {

struct MotorPins {
    gpio_num_t rpwm_gpio;
    gpio_num_t lpwm_gpio;
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;
};

struct MotorPeripheralConfig {
    MotorPins pins;
    ledc_channel_t rpwm_channel;
    ledc_channel_t lpwm_channel;
    bool invert_motor;
    bool invert_encoder;
};

// R_EN/L_EN are intentionally not managed by GPIO; connect both directly to 5 V.
// Each config owns one BTS7960 and one motor.
constexpr MotorPeripheralConfig LEFT_MOTOR{
    {GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_19, GPIO_NUM_18},
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    false,
    true,
};

constexpr MotorPeripheralConfig RIGHT_MOTOR{
    {GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_12, GPIO_NUM_13},
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    false,
    false,
};

// Quadrature x4 estimate: 11 pulses/motor revolution * gearbox ratio 34 * 4.
// Calibrate this value by rotating the output shaft exactly one revolution.
constexpr float ENCODER_COUNTS_PER_OUTPUT_REV = 1496.0f;
constexpr float WHEEL_DIAMETER_M = 0.064f;

constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t PWM_DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr unsigned int PWM_FREQUENCY_HZ = 20000;
constexpr unsigned int PWM_MAX_DUTY = (1U << PWM_DUTY_RESOLUTION) - 1U;
constexpr unsigned int PCNT_GLITCH_FILTER_NS = 1000;

}  // namespace minicar::config::motor_hardware
