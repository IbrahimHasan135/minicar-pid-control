#pragma once

#include <cstdint>

#include "driver/motor/MotorEncoderDriver.hpp"
#include "esp_err.h"
#include "service/base/Service.hpp"

namespace minicar::service {

class MotorControlService : public Service {
public:
    MotorControlService(
        driver::MotorEncoderDriver& left_driver,
        driver::MotorEncoderDriver& right_driver);

    esp_err_t init() override;
    void reset() override;

    void refreshFeedback();
    void setMotorOutputs(float left_output, float right_output);
    void stopMotor();

    float getLeftVelocityMps() const;
    float getRightVelocityMps() const;
    float getLeftRPM() const;
    float getRightRPM() const;
    float getAverageSpeedMps() const;

    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    bool isInitialized() const;

private:
    driver::MotorEncoderDriver& left_driver_;
    driver::MotorEncoderDriver& right_driver_;

    float left_velocity_mps_{0.0f};
    float right_velocity_mps_{0.0f};
    int32_t left_ticks_{0};
    int32_t right_ticks_{0};
    bool initialized_{false};
};

}  // namespace minicar::service
