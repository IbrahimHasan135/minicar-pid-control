#include "driver/motor/MotorEncoderDriver.hpp"

#include <algorithm>
#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace minicar::driver {

namespace {

constexpr const char* TAG = "MotorEncoderDriver";
constexpr int PCNT_LOW_LIMIT = -30000;
constexpr int PCNT_HIGH_LIMIT = 30000;

bool gpioIsConnected(gpio_num_t gpio)
{
    return gpio != GPIO_NUM_NC;
}

}  // namespace

MotorEncoderDriver::MotorEncoderDriver(
    const config::motor_hardware::MotorPeripheralConfig& config)
    : config_(config)
{
}

esp_err_t MotorEncoderDriver::init()
{
    if (initialized_) {
        return ESP_OK;
    }

    esp_err_t result = initPwm();
    if (result != ESP_OK) {
        stop();
        return result;
    }

    result = initEncoder();
    if (result != ESP_OK) {
        stop();
        return result;
    }

    last_rpm_ticks_ = 0;
    last_rpm_time_us_ = esp_timer_get_time();
    rpm_ = 0.0f;
    initialized_ = true;

    ESP_LOGI(
        TAG,
        "motor ready: rpwm=%d lpwm=%d enc=%d/%d",
        config_.pins.rpwm_gpio,
        config_.pins.lpwm_gpio,
        config_.pins.encoder_a_gpio,
        config_.pins.encoder_b_gpio);
    return ESP_OK;
}

void MotorEncoderDriver::setOutput(float output)
{
    if (!std::isfinite(output)) {
        stop();
        return;
    }

    output_ = std::clamp(output, -1.0f, 1.0f);
    if (writePwm(output_) != ESP_OK) {
        stop();
    }
}

int32_t MotorEncoderDriver::getTicks() const
{
    return readTicks();
}

float MotorEncoderDriver::getRPM() const
{
    if (!initialized_ || !hasEncoder()) {
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
    (void)writePwm(0.0f);
}

bool MotorEncoderDriver::hasPwm() const
{
    return gpioIsConnected(config_.pins.rpwm_gpio) &&
           gpioIsConnected(config_.pins.lpwm_gpio);
}

bool MotorEncoderDriver::hasEncoder() const
{
    return gpioIsConnected(config_.pins.encoder_a_gpio) &&
           gpioIsConnected(config_.pins.encoder_b_gpio);
}

esp_err_t MotorEncoderDriver::initPwm()
{
    using namespace config::motor_hardware;

    if (!hasPwm()) {
        ESP_LOGW(TAG, "PWM pin skipped because RPWM/LPWM is not configured");
        return ESP_OK;
    }

    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = PWM_SPEED_MODE;
    timer_config.duty_resolution = PWM_DUTY_RESOLUTION;
    timer_config.timer_num = PWM_TIMER;
    timer_config.freq_hz = PWM_FREQUENCY_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "configure LEDC timer failed");

    if (gpioIsConnected(config_.pins.r_enable_gpio) || gpioIsConnected(config_.pins.l_enable_gpio)) {
        gpio_config_t enable_config{};
        enable_config.pin_bit_mask =
            (gpioIsConnected(config_.pins.r_enable_gpio) ? (1ULL << config_.pins.r_enable_gpio) : 0ULL) |
            (gpioIsConnected(config_.pins.l_enable_gpio) ? (1ULL << config_.pins.l_enable_gpio) : 0ULL);
        enable_config.mode = GPIO_MODE_OUTPUT;
        enable_config.pull_up_en = GPIO_PULLUP_DISABLE;
        enable_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        enable_config.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&enable_config), TAG, "configure enable GPIO failed");
        if (gpioIsConnected(config_.pins.r_enable_gpio)) {
            ESP_RETURN_ON_ERROR(gpio_set_level(config_.pins.r_enable_gpio, 1), TAG, "set R_EN failed");
        }
        if (gpioIsConnected(config_.pins.l_enable_gpio)) {
            ESP_RETURN_ON_ERROR(gpio_set_level(config_.pins.l_enable_gpio, 1), TAG, "set L_EN failed");
        }
    }

    ledc_channel_config_t rpwm_config{};
    rpwm_config.gpio_num = config_.pins.rpwm_gpio;
    rpwm_config.speed_mode = PWM_SPEED_MODE;
    rpwm_config.channel = config_.rpwm_channel;
    rpwm_config.intr_type = LEDC_INTR_DISABLE;
    rpwm_config.timer_sel = PWM_TIMER;
    rpwm_config.duty = 0;
    rpwm_config.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&rpwm_config), TAG, "configure RPWM failed");

    ledc_channel_config_t lpwm_config{};
    lpwm_config.gpio_num = config_.pins.lpwm_gpio;
    lpwm_config.speed_mode = PWM_SPEED_MODE;
    lpwm_config.channel = config_.lpwm_channel;
    lpwm_config.intr_type = LEDC_INTR_DISABLE;
    lpwm_config.timer_sel = PWM_TIMER;
    lpwm_config.duty = 0;
    lpwm_config.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&lpwm_config), TAG, "configure LPWM failed");

    return writePwm(0.0f);
}

esp_err_t MotorEncoderDriver::initEncoder()
{
    using namespace config::motor_hardware;

    if (!hasEncoder()) {
        ESP_LOGW(TAG, "encoder skipped because A/B is not configured");
        return ESP_OK;
    }

    gpio_config_t input_config{};
    input_config.pin_bit_mask = (1ULL << config_.pins.encoder_a_gpio) | (1ULL << config_.pins.encoder_b_gpio);
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
    channel_a_config.edge_gpio_num = config_.pins.encoder_a_gpio;
    channel_a_config.level_gpio_num = config_.pins.encoder_b_gpio;
    result = pcnt_new_channel(encoder_unit_, &channel_a_config, &encoder_channel_a_);
    if (result != ESP_OK) {
        return result;
    }

    pcnt_chan_config_t channel_b_config{};
    channel_b_config.edge_gpio_num = config_.pins.encoder_b_gpio;
    channel_b_config.level_gpio_num = config_.pins.encoder_a_gpio;
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

    return ESP_OK;
}

esp_err_t MotorEncoderDriver::writePwm(float output)
{
    using namespace config::motor_hardware;

    if (!hasPwm()) {
        return ESP_OK;
    }

    const float signed_output = config_.invert_motor ? -output : output;
    const float magnitude = std::fabs(std::clamp(signed_output, -1.0f, 1.0f));
    const uint32_t duty = static_cast<uint32_t>(magnitude * static_cast<float>(PWM_MAX_DUTY));
    const uint32_t forward_duty = signed_output > 0.0f ? duty : 0U;
    const uint32_t reverse_duty = signed_output < 0.0f ? duty : 0U;

    ESP_RETURN_ON_ERROR(ledc_set_duty(PWM_SPEED_MODE, config_.rpwm_channel, forward_duty), TAG, "set RPWM duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(PWM_SPEED_MODE, config_.rpwm_channel), TAG, "update RPWM duty failed");
    ESP_RETURN_ON_ERROR(ledc_set_duty(PWM_SPEED_MODE, config_.lpwm_channel, reverse_duty), TAG, "set LPWM duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(PWM_SPEED_MODE, config_.lpwm_channel), TAG, "update LPWM duty failed");
    return ESP_OK;
}

int32_t MotorEncoderDriver::readTicks() const
{
    if (!initialized_ || !hasEncoder()) {
        return 0;
    }

    int count = 0;
    if (pcnt_unit_get_count(encoder_unit_, &count) != ESP_OK) {
        return 0;
    }

    return config_.invert_encoder ? -count : count;
}

}  // namespace minicar::driver
