#pragma once

#include "esp_err.h"

namespace minicar::driver {

class IMUDriver {
public:
    esp_err_t init();

    float getYawRateDps() const;
    bool isHealthy() const;
};

}  // namespace minicar::driver
