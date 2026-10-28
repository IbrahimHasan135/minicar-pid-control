#include "driver/imu/IMUDriver.hpp"

#include <cstdint>
#include <cmath>

#include "driver/gpio.h"

namespace minicar::driver {

namespace {

constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr int I2C_TIMEOUT_MS = 100;

constexpr uint8_t ACCELEROMETER_ADDRESS = 0x19;
constexpr uint8_t MAGNETOMETER_ADDRESS = 0x1E;

constexpr uint8_t CTRL_REG1_A = 0x20;
constexpr uint8_t CTRL_REG4_A = 0x23;
constexpr uint8_t OUT_X_L_A = 0x28;
constexpr uint8_t CRA_REG_M = 0x00;
constexpr uint8_t CRB_REG_M = 0x01;
constexpr uint8_t MR_REG_M = 0x02;
constexpr uint8_t OUT_X_H_M = 0x03;

constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
constexpr float ACCELEROMETER_G_PER_LSB = 0.001f;
constexpr float GAUSS_TO_MICROTESLA = 100.0f;
constexpr float MAGNETOMETER_XY_LSB_PER_GAUSS = 1100.0f;
constexpr float MAGNETOMETER_Z_LSB_PER_GAUSS = 980.0f;
constexpr float PI = 3.14159265358979323846f;

int16_t littleEndianInt16(const uint8_t low, const uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

int16_t bigEndianInt16(const uint8_t high, const uint8_t low)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace

esp_err_t IMUDriver::init()
{
    healthy_ = false;

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = I2C_SDA_PIN;
    bus_config.scl_io_num = I2C_SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t result = i2c_new_master_bus(&bus_config, &bus_);
    if (result != ESP_OK) {
        return result;
    }

    result = i2c_master_probe(bus_, ACCELEROMETER_ADDRESS, I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }
    result = i2c_master_probe(bus_, MAGNETOMETER_ADDRESS, I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    i2c_device_config_t accelerometer_config{};
    accelerometer_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    accelerometer_config.device_address = ACCELEROMETER_ADDRESS;
    accelerometer_config.scl_speed_hz = I2C_FREQUENCY_HZ;
    result = i2c_master_bus_add_device(bus_, &accelerometer_config, &accelerometer_);
    if (result != ESP_OK) {
        return result;
    }

    i2c_device_config_t magnetometer_config{};
    magnetometer_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    magnetometer_config.device_address = MAGNETOMETER_ADDRESS;
    magnetometer_config.scl_speed_hz = I2C_FREQUENCY_HZ;
    result = i2c_master_bus_add_device(bus_, &magnetometer_config, &magnetometer_);
    if (result != ESP_OK) {
        return result;
    }

    // Accelerometer: 100 Hz, XYZ enabled, high-resolution, +/-2 g, block update.
    result = writeRegister(accelerometer_, CTRL_REG1_A, 0x57);
    if (result == ESP_OK) {
        result = writeRegister(accelerometer_, CTRL_REG4_A, 0x88);
    }

    // Magnetometer: 75 Hz, +/-1.3 gauss, continuous-conversion mode.
    if (result == ESP_OK) {
        result = writeRegister(magnetometer_, CRA_REG_M, 0x18);
    }
    if (result == ESP_OK) {
        result = writeRegister(magnetometer_, CRB_REG_M, 0x20);
    }
    if (result == ESP_OK) {
        result = writeRegister(magnetometer_, MR_REG_M, 0x00);
    }

    healthy_ = (result == ESP_OK);
    return result;
}

esp_err_t IMUDriver::readAcceleration(AccelerationMps2& acceleration) const
{
    if (!healthy_ || accelerometer_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[6]{};
    const esp_err_t result = readRegisters(accelerometer_, OUT_X_L_A | 0x80, data, sizeof(data));
    if (result != ESP_OK) {
        healthy_ = false;
        return result;
    }

    const int16_t x_raw = littleEndianInt16(data[0], data[1]) >> 4;
    const int16_t y_raw = littleEndianInt16(data[2], data[3]) >> 4;
    const int16_t z_raw = littleEndianInt16(data[4], data[5]) >> 4;
    constexpr float scale = ACCELEROMETER_G_PER_LSB * STANDARD_GRAVITY_MPS2;
    acceleration = {
        static_cast<float>(x_raw) * scale,
        static_cast<float>(y_raw) * scale,
        static_cast<float>(z_raw) * scale,
    };
    return ESP_OK;
}

esp_err_t IMUDriver::readMagneticField(MagneticFieldUt& magnetic_field) const
{
    if (!healthy_ || magnetometer_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[6]{};
    const esp_err_t result = readRegisters(magnetometer_, OUT_X_H_M, data, sizeof(data));
    if (result != ESP_OK) {
        healthy_ = false;
        return result;
    }

    // LSM303DLHC magnetometer register order is X, Z, Y.
    const int16_t x_raw = bigEndianInt16(data[0], data[1]);
    const int16_t z_raw = bigEndianInt16(data[2], data[3]);
    const int16_t y_raw = bigEndianInt16(data[4], data[5]);
    magnetic_field = {
        static_cast<float>(x_raw) * GAUSS_TO_MICROTESLA / MAGNETOMETER_XY_LSB_PER_GAUSS,
        static_cast<float>(y_raw) * GAUSS_TO_MICROTESLA / MAGNETOMETER_XY_LSB_PER_GAUSS,
        static_cast<float>(z_raw) * GAUSS_TO_MICROTESLA / MAGNETOMETER_Z_LSB_PER_GAUSS,
    };
    return ESP_OK;
}

esp_err_t IMUDriver::readHeadingDeg(float& heading_deg) const
{
    MagneticFieldUt magnetic_field{};
    const esp_err_t result = readMagneticField(magnetic_field);
    if (result != ESP_OK) {
        return result;
    }

    const float calibrated_x = (magnetic_field.x - magnetic_offset_ut_.x) * magnetic_scale_.x;
    const float calibrated_y = (magnetic_field.y - magnetic_offset_ut_.y) * magnetic_scale_.y;
    if (calibrated_x == 0.0f && calibrated_y == 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    heading_deg = std::atan2(calibrated_y, calibrated_x) * (180.0f / PI);
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    return ESP_OK;
}

esp_err_t IMUDriver::setMagnetometerCalibration(const MagnetometerCalibration& calibration)
{
    const float radius_x = (calibration.maximum.x - calibration.minimum.x) * 0.5f;
    const float radius_y = (calibration.maximum.y - calibration.minimum.y) * 0.5f;
    const float radius_z = (calibration.maximum.z - calibration.minimum.z) * 0.5f;
    if (radius_x <= 0.0f || radius_y <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    magnetic_offset_ut_ = {
        (calibration.maximum.x + calibration.minimum.x) * 0.5f,
        (calibration.maximum.y + calibration.minimum.y) * 0.5f,
        (calibration.maximum.z + calibration.minimum.z) * 0.5f,
    };

    const float average_radius = (radius_x + radius_y) * 0.5f;
    magnetic_scale_ = {
        average_radius / radius_x,
        average_radius / radius_y,
        (radius_z > 0.0f) ? average_radius / radius_z : 1.0f,
    };
    return ESP_OK;
}

bool IMUDriver::isHealthy() const
{
    return healthy_;
}

esp_err_t IMUDriver::writeRegister(
    const i2c_master_dev_handle_t device,
    const uint8_t reg,
    const uint8_t value) const
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), I2C_TIMEOUT_MS);
}

esp_err_t IMUDriver::readRegisters(
    const i2c_master_dev_handle_t device,
    const uint8_t first_reg,
    uint8_t* data,
    const size_t length) const
{
    return i2c_master_transmit_receive(
        device,
        &first_reg,
        sizeof(first_reg),
        data,
        length,
        I2C_TIMEOUT_MS);
}

}  // namespace minicar::driver
