#include "service/navigation/NavigationSerialService.hpp"

#include <cmath>
#include <cstring>

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace minicar::service {

NavigationSerialService::NavigationSerialService(
    driver::NavigationSerialDriver& driver)
    : driver_(driver)
{
}

esp_err_t NavigationSerialService::init()
{
    const esp_err_t result = driver_.init();
    initialized_ = (result == ESP_OK);
    return result;
}

void NavigationSerialService::reset()
{
}

int NavigationSerialService::readBytes(std::uint8_t* buffer,
                                       std::size_t buffer_size,
                                       std::uint32_t timeout_ms)
{
    if (!initialized_) {
        return -1;
    }
    return driver_.readBytes(buffer, buffer_size, timeout_ms);
}

int NavigationSerialService::rxLevel() const
{
    return initialized_ ? driver_.rxLevel() : -1;
}

bool NavigationSerialService::parseJsonLine(
    const char* line,
    model::NavigationSample& sample) const
{
    if (line == nullptr) {
        return false;
    }

    const char* parse_end = nullptr;
    cJSON* root = cJSON_ParseWithOpts(line, &parse_end, true);
    if (root == nullptr || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return false;
    }

    model::NavigationSample parsed{};
    bool seen[4]{};
    std::size_t member_count = 0;
    bool valid = true;

    for (cJSON* item = root->child; item != nullptr; item = item->next) {
        ++member_count;
        int index = -1;
        if (item->string != nullptr && std::strlen(item->string) == 6 &&
            std::strncmp(item->string, "value", 5) == 0 &&
            item->string[5] >= '1' && item->string[5] <= '4') {
            index = item->string[5] - '1';
        }

        if (index < 0 || seen[index]) {
            valid = false;
            break;
        }
        seen[index] = true;

        if (cJSON_IsNull(item)) {
            parsed.values[index].valid = false;
        } else if (cJSON_IsNumber(item) && std::isfinite(item->valuedouble)) {
            parsed.values[index].value = item->valuedouble;
            parsed.values[index].valid = true;
        } else {
            valid = false;
            break;
        }
    }

    valid = valid && member_count == 4;
    for (bool field_seen : seen) {
        valid = valid && field_seen;
    }

    if (valid) {
        parsed.received_at_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        sample = parsed;
    }
    cJSON_Delete(root);
    return valid;
}

}  // namespace minicar::service
