#pragma once

#include "driver/imu/IMUDriver.hpp"
#include "esp_err.h"
#include "service/base/Service.hpp"
#include "service/control/PIDController.hpp"

namespace minicar::service {

class HeadingService : public Service {
public:
    explicit HeadingService(driver::IMUDriver& imu);

    esp_err_t init() override;
    void reset() override;

    void updateHeading(float dt_s);
    float getHeadingDeg() const;
    float getYawRateDps() const;
    float calculateHeadingCorrection(float target_heading_deg, float dt_s);
    void resetHeadingReference();
    void resetController();
    bool isInitialized() const;

    static float normalizeAngleDeg(float angle_deg);
    static float shortestAngleErrorDeg(float target_deg, float current_deg);

private:
    driver::IMUDriver& imu_;
    PIDController heading_pid_;

    float heading_deg_{0.0f};
    float yaw_rate_dps_{0.0f};
    bool initialized_{false};
};

}  // namespace minicar::service
