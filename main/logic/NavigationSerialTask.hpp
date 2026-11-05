#pragma once

#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "service/navigation/NavigationSerialService.hpp"

namespace minicar::logic {

class NavigationSerialTask {
public:
    explicit NavigationSerialTask(
        service::NavigationSerialService& navigation_service);

    esp_err_t init();
    esp_err_t start();

private:
    static void receiveTaskEntry(void* context);
    static void frameTaskEntry(void* context);
    void receiveTask();
    void frameTask();
    void consumeByte(char byte,
                     char* frame,
                     std::size_t& frame_length,
                     bool& discarding);

    service::NavigationSerialService& navigation_service_;
    StreamBufferHandle_t stream_buffer_{nullptr};
};

}  // namespace minicar::logic
