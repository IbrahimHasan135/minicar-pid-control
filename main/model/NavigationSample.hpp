#pragma once

#include <cstdint>

namespace minicar::model {

struct NavigationValue {
    double value{0.0};
    bool valid{false};
};

enum class NavigationSampleType : std::uint8_t {
    UNKNOWN = 0,
    RGBC,
    RF,
};

struct RfRouterSample {
    std::uint16_t short_address{0};
    std::int8_t rssi{0};
};

struct NavigationSample {
    NavigationSampleType type{NavigationSampleType::UNKNOWN};
    NavigationValue values[4]{};
    RfRouterSample routers[4]{};
    std::uint8_t router_count{0};
    std::uint32_t received_at_ms{0};
};

}  // namespace minicar::model
