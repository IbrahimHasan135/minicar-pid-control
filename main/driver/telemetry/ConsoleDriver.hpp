#pragma once

#include <cstddef>

#include "esp_err.h"

namespace minicar::driver {

class ConsoleDriver {
public:
    esp_err_t init();
    esp_err_t writeLine(const char* line, size_t length);
};

}  // namespace minicar::driver
