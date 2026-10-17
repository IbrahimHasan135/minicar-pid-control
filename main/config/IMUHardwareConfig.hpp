#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"

namespace minicar::config::imu_hardware {

constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t SDA_GPIO = GPIO_NUM_21;
constexpr gpio_num_t SCL_GPIO = GPIO_NUM_22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr int I2C_TIMEOUT_MS = 50;

// LSM303DLHC accelerometer can appear at 0x18 or 0x19 depending on SA0_A.
constexpr uint8_t ACCEL_ADDRESS_PRIMARY = 0x19;
constexpr uint8_t ACCEL_ADDRESS_SECONDARY = 0x18;
constexpr uint8_t MAG_ADDRESS = 0x1E;
constexpr uint8_t ACCEL_WHO_AM_I = 0x33;

// Hard-iron offset and soft-iron scale must be calibrated after the GY-511 is
// mounted on the vehicle. Defaults preserve raw axes for initial bring-up.
constexpr float MAG_OFFSET_X = 0.0f;
constexpr float MAG_OFFSET_Y = 0.0f;
constexpr float MAG_OFFSET_Z = 0.0f;
constexpr float MAG_SCALE_X = 1.0f;
constexpr float MAG_SCALE_Y = 1.0f;
constexpr float MAG_SCALE_Z = 1.0f;

// Adjust these if the board orientation makes clockwise vehicle rotation
// produce a negative yaw rate.
constexpr bool INVERT_HEADING = false;
constexpr float MAGNETIC_DECLINATION_DEG = 0.0f;

// Low-pass coefficient for yaw rate derived from magnetic heading delta.
// 1.0 means no filtering; smaller values are smoother but react more slowly.
constexpr float YAW_RATE_FILTER_ALPHA = 0.25f;
constexpr float MAX_VALID_YAW_RATE_DPS = 720.0f;

}  // namespace minicar::config::imu_hardware
