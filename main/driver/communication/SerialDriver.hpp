#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace minicar::driver {

class SerialDriver {
public:
    esp_err_t init();
    bool readLine(char* buffer, size_t buffer_size, uint32_t timeout_ms);
};

}  // namespace minicar::driver
