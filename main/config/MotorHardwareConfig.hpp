#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

namespace minicar::config::motor_hardware {

struct MotorPins {
    gpio_num_t rpwm_gpio;
    gpio_num_t lpwm_gpio;
    gpio_num_t r_enable_gpio;
    gpio_num_t l_enable_gpio;
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;
};

struct MotorPeripheralConfig {
    MotorPins pins;
    ledc_channel_t rpwm_channel;
    ledc_channel_t lpwm_channel;
    float encoder_counts_per_output_rev;
    bool invert_motor;
    bool invert_encoder;
};

// PCNT menghitung rising + falling edge pada kanal A dan B (quadrature x4).
// Nilai 1496 adalah titik awal: 11 pulse/motor-rev * rasio gearbox 34 * x4.
// Kalibrasikan dengan memutar output shaft tepat satu putaran dan menghitung
// perubahan ticks aktual.
constexpr float ENCODER_COUNTS_PER_OUTPUT_REV_ESTIMATE = 1496.0f;

// BTS7960 bench pinout. WAJIB cocokkan dengan wiring fisik sebelum flash.
// Gunakan GPIO_NUM_NC untuk pin yang belum dipasang.
constexpr MotorPeripheralConfig LEFT_MOTOR{
    MotorPins{
        GPIO_NUM_25,  // RPWM
        GPIO_NUM_26,  // LPWM
        GPIO_NUM_27,  // R_EN
        GPIO_NUM_14,  // L_EN
        GPIO_NUM_32,  // Encoder A
        GPIO_NUM_33,  // Encoder B
    },
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    ENCODER_COUNTS_PER_OUTPUT_REV_ESTIMATE,
    false,
    false,
};

// Right motor belum difokuskan pada bench awal. Isi pin ini saat BTS7960 kanan
// dan encoder kanan sudah benar-benar dipasang.
constexpr MotorPeripheralConfig RIGHT_MOTOR{
    MotorPins{
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
    },
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    ENCODER_COUNTS_PER_OUTPUT_REV_ESTIMATE,
    false,
    false,
};

constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t PWM_DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr unsigned int PWM_FREQUENCY_HZ = 20000;
constexpr unsigned int PWM_MAX_DUTY = (1U << PWM_DUTY_RESOLUTION) - 1U;

// Abaikan pulsa yang lebih pendek dari nilai ini. Naikkan hanya jika ada noise.
constexpr unsigned int PCNT_GLITCH_FILTER_NS = 1000;

}  // namespace minicar::config::motor_hardware
