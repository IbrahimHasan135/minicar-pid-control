#include "driver/motor/MotorEncoderDriver.hpp"

#include <cmath>

#include "config/MotorHardwareConfig.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace minicar::driver {

namespace {

constexpr const char* TAG = "MotorEncoderDriver";
constexpr int PCNT_LOW_LIMIT = -30000;
constexpr int PCNT_HIGH_LIMIT = 30000;

}  // namespace

esp_err_t MotorEncoderDriver::init()
{
    using namespace config::motor_hardware;

    if (initialized_) {
        return ESP_OK;
    }

    gpio_config_t input_config{};
    input_config.pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO);
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pull_up_en = GPIO_PULLUP_ENABLE;
    input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_config.intr_type = GPIO_INTR_DISABLE;
    esp_err_t result = gpio_config(&input_config);
    if (result != ESP_OK) {
        return result;
    }

    pcnt_unit_config_t unit_config{};
    unit_config.low_limit = PCNT_LOW_LIMIT;
    unit_config.high_limit = PCNT_HIGH_LIMIT;
    unit_config.flags.accum_count = true;
    result = pcnt_new_unit(&unit_config, &encoder_unit_);
    if (result != ESP_OK) {
        return result;
    }

    pcnt_glitch_filter_config_t filter_config{};
    filter_config.max_glitch_ns = PCNT_GLITCH_FILTER_NS;
    result = pcnt_unit_set_glitch_filter(encoder_unit_, &filter_config);
    if (result != ESP_OK) {
        return result;
    }

    pcnt_chan_config_t channel_a_config{};
    channel_a_config.edge_gpio_num = ENCODER_A_GPIO;
    channel_a_config.level_gpio_num = ENCODER_B_GPIO;
    result = pcnt_new_channel(encoder_unit_, &channel_a_config, &encoder_channel_a_);
    if (result != ESP_OK) {
        return result;
    }

    pcnt_chan_config_t channel_b_config{};
    channel_b_config.edge_gpio_num = ENCODER_B_GPIO;
    channel_b_config.level_gpio_num = ENCODER_A_GPIO;
    result = pcnt_new_channel(encoder_unit_, &channel_b_config, &encoder_channel_b_);
    if (result != ESP_OK) {
        return result;
    }

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            encoder_channel_a_,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG,
        "set channel A edge action failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            encoder_channel_a_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "set channel A level action failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            encoder_channel_b_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG,
        "set channel B edge action failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            encoder_channel_b_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "set channel B level action failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder_unit_, PCNT_LOW_LIMIT), TAG, "add low watch point failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder_unit_, PCNT_HIGH_LIMIT), TAG, "add high watch point failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(encoder_unit_), TAG, "enable PCNT failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(encoder_unit_), TAG, "clear PCNT failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(encoder_unit_), TAG, "start PCNT failed");

    last_rpm_ticks_ = 0;
    last_rpm_time_us_ = esp_timer_get_time();
    rpm_ = 0.0f;
    initialized_ = true;

    ESP_LOGI(TAG, "quadrature encoder ready on GPIO %d/%d", ENCODER_A_GPIO, ENCODER_B_GPIO);
    return ESP_OK;
}

void MotorEncoderDriver::setOutput(float output)
{
    // TODO(Aranda): apply this motor output to hardware.
    output_ = output;
}

int32_t MotorEncoderDriver::getTicks() const
{
    return readTicks();
}

float MotorEncoderDriver::getRPM() const
{
    if (!initialized_) {
        return 0.0f;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - last_rpm_time_us_;
    if (elapsed_us < 100000) {
        return rpm_;
    }

    const int32_t ticks = readTicks();
    const int32_t delta_ticks = ticks - last_rpm_ticks_;
    const float elapsed_s = static_cast<float>(elapsed_us) / 1000000.0f;
    rpm_ = (static_cast<float>(delta_ticks) * 60.0f) /
           (config::motor_hardware::ENCODER_COUNTS_PER_OUTPUT_REV * elapsed_s);
    last_rpm_ticks_ = ticks;
    last_rpm_time_us_ = now_us;
    return rpm_;
}

void MotorEncoderDriver::stop()
{
    output_ = 0.0f;
    // TODO(Aranda): force this motor output to a safe stop state.
}

int32_t MotorEncoderDriver::readTicks() const
{
    if (!initialized_) {
        return 0;
    }

    int count = 0;
    if (pcnt_unit_get_count(encoder_unit_, &count) != ESP_OK) {
        return 0;
    }

    return config::motor_hardware::INVERT_ENCODER_DIRECTION ? -count : count;
}

}  // namespace minicar::driver
