#pragma once

#include "driver/gpio.h"

namespace minicar::config::motor_hardware {

// Bench-test sementara untuk satu encoder quadrature.
// WAJIB cocokkan dengan wiring fisik sebelum flash.
constexpr gpio_num_t ENCODER_A_GPIO = GPIO_NUM_32;
constexpr gpio_num_t ENCODER_B_GPIO = GPIO_NUM_33;

// PCNT menghitung rising + falling edge pada kanal A dan B (quadrature x4).
// Nilai 1496 hanya perkiraan awal: 11 pulse/motor-rev * rasio gearbox 34 * x4.
// Ukur satu putaran output shaft, lalu ganti dengan count aktual motor Anda.
constexpr float ENCODER_COUNTS_PER_OUTPUT_REV = 1496.0f;

// Balik menjadi true jika putaran maju menghasilkan count negatif.
constexpr bool INVERT_ENCODER_DIRECTION = false;

// Abaikan pulsa yang lebih pendek dari nilai ini. Naikkan hanya jika ada noise.
constexpr unsigned int PCNT_GLITCH_FILTER_NS = 1000;

}  // namespace minicar::config::motor_hardware
