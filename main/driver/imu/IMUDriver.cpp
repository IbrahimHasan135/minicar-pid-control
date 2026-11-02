#include "driver/imu/IMUDriver.hpp"

namespace minicar::driver {

esp_err_t IMUDriver::init()
{
    // TODO(Aranda): configure I2C/SPI and initialize the selected IMU.
    return ESP_ERR_NOT_SUPPORTED;
}

float IMUDriver::getYawRateDps() const
{
    // TODO(Aranda): return calibrated gyro yaw rate in degree/second.
    return 0.0f;
}

bool IMUDriver::isHealthy() const
{
    // TODO(Aranda): report real IMU health after hardware init and read checks.
    return false;
}

}  // namespace minicar::driver
