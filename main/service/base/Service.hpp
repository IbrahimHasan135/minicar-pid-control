#pragma once

#include "esp_err.h"

namespace minicar::service {

class Service {
public:
    virtual esp_err_t init() = 0;
    virtual void reset() = 0;
    virtual ~Service() = default;
};

}  // namespace minicar::service
