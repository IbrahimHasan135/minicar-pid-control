#pragma once

#include "freertos/FreeRTOS.h"
#include "model/NavigationSample.hpp"

namespace minicar::service {

// Thread-safe handoff between the UART parser and the motion controller.
class RgbcSampleStore {
public:
    void update(const model::NavigationSample& sample);
    bool latest(model::NavigationSample& sample) const;

private:
    mutable portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;
    model::NavigationSample latest_sample_{};
    bool has_sample_{false};
};

}  // namespace minicar::service
