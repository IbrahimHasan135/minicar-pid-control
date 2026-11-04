#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace minicar::driver {

class NavigationSerialDriver {
public:
    esp_err_t init();
    int readBytes(std::uint8_t* buffer,
                  std::size_t buffer_size,
                  std::uint32_t timeout_ms);

private:
    bool initialized_{false};
};

}  // namespace minicar::driver
