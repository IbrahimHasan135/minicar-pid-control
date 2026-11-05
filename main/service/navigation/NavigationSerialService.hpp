#pragma once

#include "driver/navigation/NavigationSerialDriver.hpp"
#include "esp_err.h"
#include "model/NavigationSample.hpp"
#include "service/base/Service.hpp"

namespace minicar::service {

class NavigationSerialService : public Service {
public:
    explicit NavigationSerialService(driver::NavigationSerialDriver& driver);

    esp_err_t init() override;
    void reset() override;
    int readBytes(std::uint8_t* buffer,
                  std::size_t buffer_size,
                  std::uint32_t timeout_ms);
    int rxLevel() const;
    bool parseJsonLine(const char* line, model::NavigationSample& sample) const;

private:
    driver::NavigationSerialDriver& driver_;
    bool initialized_{false};
};

}  // namespace minicar::service
