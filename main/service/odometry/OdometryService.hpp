#pragma once

#include <cstdint>

#include "esp_err.h"
#include "model/Pose2D.hpp"
#include "service/base/Service.hpp"

namespace minicar::service {

class OdometryService : public Service {
public:
    esp_err_t init() override;
    void reset() override;

    void updateOdometry(int32_t left_ticks, int32_t right_ticks, float heading_deg);
    float getTravelledDistanceM() const;
    model::Pose2D getPose() const;
    void resetDistanceReference();

private:
    float ticksToMeters(int32_t ticks) const;

    model::Pose2D pose_{};
    float travelled_distance_m_{0.0f};
    int32_t last_left_ticks_{0};
    int32_t last_right_ticks_{0};
    bool has_last_ticks_{false};
};

}  // namespace minicar::service
