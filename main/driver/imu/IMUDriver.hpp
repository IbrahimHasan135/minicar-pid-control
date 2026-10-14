#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace minicar::driver {

class IMUDriver {
public:
    esp_err_t init();

    float getYawRateDps() const;
    bool isHealthy() const;

private:
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    esp_err_t addDevice(uint8_t address, i2c_master_dev_handle_t& device);
    esp_err_t writeRegister(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value) const;
    esp_err_t readRegisters(
        i2c_master_dev_handle_t device,
        uint8_t start_reg,
        uint8_t* data,
        size_t length) const;
    esp_err_t readAcceleration(Vector3& acceleration_g) const;
    esp_err_t readMagneticField(Vector3& magnetic) const;
    esp_err_t readHeadingDeg(float& heading_deg) const;
    void cleanup();

    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t accel_device_{nullptr};
    i2c_master_dev_handle_t mag_device_{nullptr};

    mutable int64_t last_sample_time_us_{0};
    mutable float last_heading_deg_{0.0f};
    mutable float filtered_yaw_rate_dps_{0.0f};
    mutable bool has_heading_sample_{false};
    mutable bool healthy_{false};
};

}  // namespace minicar::driver
