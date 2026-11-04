#pragma once

#include "driver/gpio.h"

namespace minicar::config::imu_hardware {

// Reserved wiring for the future IMU implementation.
constexpr gpio_num_t SDA_GPIO = GPIO_NUM_17;
constexpr gpio_num_t SCL_GPIO = GPIO_NUM_16;

}  // namespace minicar::config::imu_hardware
