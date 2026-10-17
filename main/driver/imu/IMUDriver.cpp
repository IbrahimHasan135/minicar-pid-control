#include "driver/imu/IMUDriver.hpp"

#include <cmath>

#include "config/IMUHardwareConfig.hpp"
#include "esp_log.h"
#include "esp_timer.h"

namespace minicar::driver {

namespace {

constexpr const char* TAG = "IMUDriver";
constexpr float PI = 3.14159265358979323846f;

constexpr uint8_t WHO_AM_I_A = 0x0F;
constexpr uint8_t CTRL_REG1_A = 0x20;
constexpr uint8_t CTRL_REG4_A = 0x23;
constexpr uint8_t OUT_X_L_A = 0x28;
constexpr uint8_t ACCEL_AUTO_INCREMENT = 0x80;

constexpr uint8_t CRA_REG_M = 0x00;
constexpr uint8_t CRB_REG_M = 0x01;
constexpr uint8_t MR_REG_M = 0x02;
constexpr uint8_t OUT_X_H_M = 0x03;

float normalizeAngleDeg(float angle_deg)
{
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

int16_t littleEndianInt16(const uint8_t low, const uint8_t high)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(low) |
        (static_cast<uint16_t>(high) << 8U));
}

int16_t bigEndianInt16(const uint8_t high, const uint8_t low)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(high) << 8U) |
        static_cast<uint16_t>(low));
}

}  // namespace

esp_err_t IMUDriver::init()
{
    using namespace config::imu_hardware;

    if (healthy_) {
        return ESP_OK;
    }

    cleanup();

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_PORT;
    bus_config.sda_io_num = SDA_GPIO;
    bus_config.scl_io_num = SCL_GPIO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t result = i2c_new_master_bus(&bus_config, &bus_);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "create I2C bus failed: %s", esp_err_to_name(result));
        cleanup();
        return result;
    }

    uint8_t accel_address = ACCEL_ADDRESS_PRIMARY;
    result = i2c_master_probe(bus_, accel_address, I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        accel_address = ACCEL_ADDRESS_SECONDARY;
        result = i2c_master_probe(bus_, accel_address, I2C_TIMEOUT_MS);
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "LSM303 accelerometer not found at 0x18/0x19");
        cleanup();
        return ESP_ERR_NOT_FOUND;
    }

    result = i2c_master_probe(bus_, MAG_ADDRESS, I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "LSM303 magnetometer not found at 0x1E");
        cleanup();
        return ESP_ERR_NOT_FOUND;
    }

    result = addDevice(accel_address, accel_device_);
    if (result == ESP_OK) {
        result = addDevice(MAG_ADDRESS, mag_device_);
    }

    uint8_t who_am_i = 0;
    if (result == ESP_OK) {
        result = readRegisters(accel_device_, WHO_AM_I_A, &who_am_i, 1);
    }
    if (result == ESP_OK && who_am_i != ACCEL_WHO_AM_I) {
        ESP_LOGE(TAG, "unexpected accelerometer WHO_AM_I=0x%02X", who_am_i);
        result = ESP_ERR_INVALID_RESPONSE;
    }

    // Accelerometer: 100 Hz, XYZ enabled, high-resolution, +/-2 g, BDU.
    if (result == ESP_OK) {
        result = writeRegister(accel_device_, CTRL_REG1_A, 0x57);
    }
    if (result == ESP_OK) {
        result = writeRegister(accel_device_, CTRL_REG4_A, 0x88);
    }

    // Magnetometer: 75 Hz, +/-1.3 gauss, continuous conversion.
    if (result == ESP_OK) {
        result = writeRegister(mag_device_, CRA_REG_M, 0x18);
    }
    if (result == ESP_OK) {
        result = writeRegister(mag_device_, CRB_REG_M, 0x20);
    }
    if (result == ESP_OK) {
        result = writeRegister(mag_device_, MR_REG_M, 0x00);
    }

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "LSM303 configuration failed: %s", esp_err_to_name(result));
        cleanup();
        return result;
    }

    last_heading_deg_ = 0.0f;
    last_sample_time_us_ = 0;
    filtered_yaw_rate_dps_ = 0.0f;
    has_heading_sample_ = false;
    healthy_ = true;

    ESP_LOGI(
        TAG,
        "GY-511/LSM303 ready: SDA=%d SCL=%d accel=0x%02X mag=0x%02X",
        SDA_GPIO,
        SCL_GPIO,
        accel_address,
        MAG_ADDRESS);
    return ESP_OK;
}

float IMUDriver::getYawRateDps() const
{
    using namespace config::imu_hardware;

    if (!healthy_) {
        return 0.0f;
    }

    float heading_deg = 0.0f;
    if (readHeadingDeg(heading_deg) != ESP_OK) {
        healthy_ = false;
        filtered_yaw_rate_dps_ = 0.0f;
        ESP_LOGE(TAG, "heading read failed; IMU marked unhealthy");
        return 0.0f;
    }

    const int64_t now_us = esp_timer_get_time();
    if (!has_heading_sample_) {
        last_heading_deg_ = heading_deg;
        last_sample_time_us_ = now_us;
        has_heading_sample_ = true;
        return 0.0f;
    }

    const int64_t elapsed_us = now_us - last_sample_time_us_;
    if (elapsed_us <= 0) {
        return filtered_yaw_rate_dps_;
    }

    const float dt_s = static_cast<float>(elapsed_us) / 1000000.0f;
    const float heading_delta_deg = normalizeAngleDeg(heading_deg - last_heading_deg_);
    float yaw_rate_dps = heading_delta_deg / dt_s;
    if (!std::isfinite(yaw_rate_dps) || std::fabs(yaw_rate_dps) > MAX_VALID_YAW_RATE_DPS) {
        yaw_rate_dps = 0.0f;
    }

    filtered_yaw_rate_dps_ +=
        YAW_RATE_FILTER_ALPHA * (yaw_rate_dps - filtered_yaw_rate_dps_);
    last_heading_deg_ = heading_deg;
    last_sample_time_us_ = now_us;
    return filtered_yaw_rate_dps_;
}

bool IMUDriver::isHealthy() const
{
    return healthy_;
}

esp_err_t IMUDriver::addDevice(uint8_t address, i2c_master_dev_handle_t& device)
{
    using namespace config::imu_hardware;

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = address;
    device_config.scl_speed_hz = I2C_FREQUENCY_HZ;
    return i2c_master_bus_add_device(bus_, &device_config, &device);
}

esp_err_t IMUDriver::writeRegister(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t value) const
{
    const uint8_t payload[]{reg, value};
    return i2c_master_transmit(
        device,
        payload,
        sizeof(payload),
        config::imu_hardware::I2C_TIMEOUT_MS);
}

esp_err_t IMUDriver::readRegisters(
    i2c_master_dev_handle_t device,
    uint8_t start_reg,
    uint8_t* data,
    size_t length) const
{
    return i2c_master_transmit_receive(
        device,
        &start_reg,
        1,
        data,
        length,
        config::imu_hardware::I2C_TIMEOUT_MS);
}

esp_err_t IMUDriver::readAcceleration(Vector3& acceleration_g) const
{
    uint8_t data[6]{};
    const esp_err_t result = readRegisters(
        accel_device_,
        OUT_X_L_A | ACCEL_AUTO_INCREMENT,
        data,
        sizeof(data));
    if (result != ESP_OK) {
        return result;
    }

    // High-resolution +/-2 g data is left-justified; divide by 16 to obtain
    // the signed 12-bit value. Sensitivity is 1 mg/LSB.
    acceleration_g.x = static_cast<float>(littleEndianInt16(data[0], data[1]) >> 4) * 0.001f;
    acceleration_g.y = static_cast<float>(littleEndianInt16(data[2], data[3]) >> 4) * 0.001f;
    acceleration_g.z = static_cast<float>(littleEndianInt16(data[4], data[5]) >> 4) * 0.001f;
    return ESP_OK;
}

esp_err_t IMUDriver::readMagneticField(Vector3& magnetic) const
{
    using namespace config::imu_hardware;

    uint8_t data[6]{};
    const esp_err_t result = readRegisters(mag_device_, OUT_X_H_M, data, sizeof(data));
    if (result != ESP_OK) {
        return result;
    }

    // Magnetometer register order is X, Z, Y and is big-endian.
    const float raw_x = static_cast<float>(bigEndianInt16(data[0], data[1]));
    const float raw_z = static_cast<float>(bigEndianInt16(data[2], data[3]));
    const float raw_y = static_cast<float>(bigEndianInt16(data[4], data[5]));

    magnetic.x = (raw_x - MAG_OFFSET_X) * MAG_SCALE_X;
    magnetic.y = (raw_y - MAG_OFFSET_Y) * MAG_SCALE_Y;
    magnetic.z = (raw_z - MAG_OFFSET_Z) * MAG_SCALE_Z;
    return ESP_OK;
}

esp_err_t IMUDriver::readHeadingDeg(float& heading_deg) const
{
    using namespace config::imu_hardware;

    Vector3 acceleration{};
    Vector3 magnetic{};
    esp_err_t result = readAcceleration(acceleration);
    if (result != ESP_OK) {
        return result;
    }
    result = readMagneticField(magnetic);
    if (result != ESP_OK) {
        return result;
    }

    const float acceleration_norm = std::sqrt(
        acceleration.x * acceleration.x +
        acceleration.y * acceleration.y +
        acceleration.z * acceleration.z);
    const float magnetic_norm = std::sqrt(
        magnetic.x * magnetic.x +
        magnetic.y * magnetic.y +
        magnetic.z * magnetic.z);
    if (acceleration_norm < 0.1f || magnetic_norm < 1.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float ax = acceleration.x / acceleration_norm;
    const float ay = acceleration.y / acceleration_norm;
    const float az = acceleration.z / acceleration_norm;
    const float roll_rad = std::atan2(ay, az);
    const float pitch_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    const float cos_roll = std::cos(roll_rad);
    const float sin_roll = std::sin(roll_rad);
    const float cos_pitch = std::cos(pitch_rad);
    const float sin_pitch = std::sin(pitch_rad);

    const float horizontal_x = magnetic.x * cos_pitch + magnetic.z * sin_pitch;
    const float horizontal_y =
        magnetic.x * sin_roll * sin_pitch +
        magnetic.y * cos_roll -
        magnetic.z * sin_roll * cos_pitch;

    float measured_heading_deg = std::atan2(horizontal_y, horizontal_x) * (180.0f / PI);
    if (INVERT_HEADING) {
        measured_heading_deg = -measured_heading_deg;
    }
    heading_deg = normalizeAngleDeg(measured_heading_deg + MAGNETIC_DECLINATION_DEG);
    return std::isfinite(heading_deg) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

void IMUDriver::cleanup()
{
    healthy_ = false;
    has_heading_sample_ = false;
    filtered_yaw_rate_dps_ = 0.0f;

    if (mag_device_ != nullptr) {
        (void)i2c_master_bus_rm_device(mag_device_);
        mag_device_ = nullptr;
    }
    if (accel_device_ != nullptr) {
        (void)i2c_master_bus_rm_device(accel_device_);
        accel_device_ = nullptr;
    }
    if (bus_ != nullptr) {
        (void)i2c_del_master_bus(bus_);
        bus_ = nullptr;
    }
}

}  // namespace minicar::driver
