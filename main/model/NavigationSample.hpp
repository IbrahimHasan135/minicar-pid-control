#pragma once

#include <cstdint>

namespace minicar::model {

struct NavigationValue {
    double value{0.0};
    bool valid{false};
};

struct NavigationSample {
    NavigationValue values[4]{};
    std::uint32_t received_at_ms{0};
};

}  // namespace minicar::model
