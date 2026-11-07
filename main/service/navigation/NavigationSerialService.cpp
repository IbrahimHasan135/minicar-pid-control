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
    bool valid = false;
    cJSON* type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (cJSON_IsString(type) && type->valuestring != nullptr &&
        std::strcmp(type->valuestring, "rgbc") == 0 &&
        cJSON_GetArraySize(root) == 5) {
        constexpr const char* fields[4] = {"R", "G", "B", "C"};
        valid = true;
        parsed.type = model::NavigationSampleType::RGBC;
        for (std::size_t i = 0; i < 4; ++i) {
            cJSON* value = cJSON_GetObjectItemCaseSensitive(root, fields[i]);
            if (!cJSON_IsNumber(value) ||
                !std::isfinite(value->valuedouble)) {
                valid = false;
                break;
            }
            parsed.values[i].value = value->valuedouble;
            parsed.values[i].valid = true;
        }
    } else if (cJSON_IsString(type) && type->valuestring != nullptr &&
               std::strcmp(type->valuestring, "rf") == 0 &&
               cJSON_GetArraySize(root) == 2) {
        cJSON* routers = cJSON_GetObjectItemCaseSensitive(root, "routers");
        const int router_count = cJSON_GetArraySize(routers);
        valid = cJSON_IsArray(routers) && router_count >= 0 && router_count <= 4;
        parsed.type = model::NavigationSampleType::RF;
        for (int i = 0; valid && i < router_count; ++i) {
            cJSON* router = cJSON_GetArrayItem(routers, i);
            cJSON* short_address =
                cJSON_GetObjectItemCaseSensitive(router, "short");
            cJSON* rssi = cJSON_GetObjectItemCaseSensitive(router, "rssi");
            valid = cJSON_IsObject(router) && cJSON_GetArraySize(router) == 2 &&
                    cJSON_IsNumber(short_address) && cJSON_IsNumber(rssi) &&
                    std::isfinite(short_address->valuedouble) &&
                    std::isfinite(rssi->valuedouble) &&
                    std::floor(short_address->valuedouble) ==
                        short_address->valuedouble &&
                    short_address->valuedouble >= 0.0 &&
                    short_address->valuedouble <= 65535.0 &&
                    std::floor(rssi->valuedouble) == rssi->valuedouble &&
                    rssi->valuedouble >= -128.0 && rssi->valuedouble <= 127.0;
            if (!valid) {
                break;
            }
            const auto parsed_short = static_cast<std::uint16_t>(
                short_address->valuedouble);
            for (int j = 0; j < i; ++j) {
                if (parsed.routers[j].short_address == parsed_short) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                break;
            }
            parsed.routers[i].short_address = parsed_short;
            parsed.routers[i].rssi =
                static_cast<std::int8_t>(rssi->valuedouble);
            ++parsed.router_count;
        }
    }

    if (valid) {
        parsed.received_at_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        sample = parsed;
    }
    cJSON_Delete(root);
    return valid;
}

}  // namespace minicar::service
