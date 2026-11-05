#pragma once

#include "esp_err.h"
#include "model/NetworkSettings.hpp"

namespace minicar::service {

class NetworkSettingsService {
public:
    esp_err_t init();
    const model::NetworkSettings& settings() const;
    esp_err_t save(const model::NetworkSettings& settings);

private:
    model::NetworkSettings settings_{};
};

}  // namespace minicar::service
