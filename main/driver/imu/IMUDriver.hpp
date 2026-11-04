#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace minicar::driver {

struct AccelerationMps2 {
    float x;
    float y;
    float z;
};

struct MagneticFieldUt {
    float x;
    float y;
    float z;
};

struct MagnetometerCalibration {
    MagneticFieldUt minimum;
    MagneticFieldUt maximum;
};

class IMUDriver {
public:
    esp_err_t init();

    esp_err_t readAcceleration(AccelerationMps2& acceleration) const;
    esp_err_t readMagneticField(MagneticFieldUt& magnetic_field) const;
    esp_err_t readHeadingDeg(float& heading_deg) const;
    esp_err_t setMagnetometerCalibration(const MagnetometerCalibration& calibration);

    bool isHealthy() const;

private:
    esp_err_t writeRegister(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value) const;
    esp_err_t readRegisters(
        i2c_master_dev_handle_t device,
        uint8_t first_reg,
        uint8_t* data,
        size_t length) const;

    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t accelerometer_{nullptr};
    i2c_master_dev_handle_t magnetometer_{nullptr};
    MagneticFieldUt magnetic_offset_ut_{};
    MagneticFieldUt magnetic_scale_{1.0f, 1.0f, 1.0f};
    mutable bool healthy_{false};
};

}  // namespace minicar::driver
