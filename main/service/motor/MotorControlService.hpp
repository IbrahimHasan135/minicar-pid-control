#pragma once

#include <cstdint>

#include "driver/motor/MotorEncoderDriver.hpp"
#include "esp_err.h"
#include "service/base/Service.hpp"
#include "service/control/PIDController.hpp"

namespace minicar::service {

class MotorControlService : public Service {
public:
    explicit MotorControlService(driver::MotorEncoderDriver& driver);

    esp_err_t init() override;
    void reset() override;

    void refreshFeedback();
    void setVelocityTargets(float left_mps, float right_mps);
    void applyControl(float dt_s);
    void stopMotor();

    float getLeftVelocityMps() const;
    float getRightVelocityMps() const;
    float getAverageSpeedMps() const;

    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    void resetControllers();
    bool isInitialized() const;

private:
    float rpmToMps(float rpm) const;
    float clampWheelSpeed(float speed_mps) const;

    driver::MotorEncoderDriver& driver_;
    PIDController left_speed_pid_;
    PIDController right_speed_pid_;

    float left_target_mps_{0.0f};
    float right_target_mps_{0.0f};
    float left_velocity_mps_{0.0f};
    float right_velocity_mps_{0.0f};
    int32_t left_ticks_{0};
    int32_t right_ticks_{0};
    bool initialized_{false};
};

}  // namespace minicar::service
