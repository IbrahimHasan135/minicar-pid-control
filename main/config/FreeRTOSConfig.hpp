#pragma once

#include <cstdint>

namespace minicar::config::rtos {

// Run the motor feedback loop at 200 Hz for faster encoder/PID response.
constexpr uint32_t CONTROL_PERIOD_MS = 5;
constexpr float CONTROL_PERIOD_S = 0.005f;

constexpr uint32_t CONTROL_TASK_STACK_WORDS = 4096;
constexpr uint32_t MOTION_TASK_STACK_WORDS = 4096;
constexpr uint32_t SERIAL_TASK_STACK_WORDS = 4096;
constexpr uint32_t TELEMETRY_TASK_STACK_WORDS = 4096;

constexpr int CONTROL_TASK_PRIORITY = 10;
constexpr int MOTION_TASK_PRIORITY = 7;
constexpr int SERIAL_TASK_PRIORITY = 5;
constexpr int TELEMETRY_TASK_PRIORITY = 2;

constexpr uint32_t MOTION_QUEUE_LENGTH = 12;
constexpr uint32_t TELEMETRY_QUEUE_LENGTH = 32;
constexpr uint32_t MOTION_TASK_IDLE_DELAY_MS = 20;
constexpr uint32_t SERIAL_TASK_IDLE_DELAY_MS = 20;

}  // namespace minicar::config::rtos
