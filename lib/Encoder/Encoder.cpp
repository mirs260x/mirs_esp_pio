#include "Encoder.hpp"

#ifdef ARDUINO

#include "esp_err.h"

namespace {
constexpr uint32_t kPcntGlitchFilterNs = 1250;
}

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a), pin_b_(pin_b)
{
}

void Encoder::begin() {
    if (initialized_) {
        return;
    }

    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);

    const pcnt_unit_config_t unit_config = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .intr_priority = 0,
        .flags = {.accum_count = 1},
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit_));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, PCNT_LIMIT));

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = kPcntGlitchFilterNs,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit_, &filter_config));

    // A相エッジを数え、B相がLOWの間は方向を反転する
    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = pin_a_,
        .level_gpio_num = pin_b_,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit_, &channel_a_config, &channel_a_));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        channel_a_, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        channel_a_, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // B相も計数し、A相がHIGHの間は方向を反転してX4計数する
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = pin_b_,
        .level_gpio_num = pin_a_,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit_, &channel_b_config, &channel_b_));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        channel_b_, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        channel_b_, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit_));
    ESP_ERROR_CHECK(pcnt_unit_start(unit_));
    initialized_ = true;
}

int64_t Encoder::getCount() const {
    if (!initialized_) {
        return 0;
    }

    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit_, &count));

    // The driver accumulator is int-sized; extend it to int64 across its wrap.
    int64_t delta = static_cast<int64_t>(count) - last_pcnt_count_;
    if (delta > INT32_MAX) {
        delta -= (INT64_C(1) << 32);
    } else if (delta < INT32_MIN) {
        delta += (INT64_C(1) << 32);
    }
    extended_count_ += delta;
    last_pcnt_count_ = static_cast<int32_t>(count);
    return extended_count_;
}

void Encoder::reset() {
    if (!initialized_) {
        return;
    }

    ESP_ERROR_CHECK(pcnt_unit_stop(unit_));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_));
    extended_count_ = 0;
    last_pcnt_count_ = 0;
    ESP_ERROR_CHECK(pcnt_unit_start(unit_));
}

#else

// ホストテスト・シミュレーション環境用（直交4逓倍デコード）
namespace {
constexpr int8_t kStep[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

static void host_isr(void *arg) {
    static_cast<Encoder *>(arg)->handleHostEdge();
}
}

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b, int unit)
    : pin_a_(pin_a), pin_b_(pin_b), unit_(unit)
{
}

void Encoder::begin() {
    if (initialized_) {
        return;
    }

    initialized_ = true;
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(pin_a_), host_isr, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pin_b_), host_isr, this, CHANGE);
    const uint8_t a = (digitalRead(pin_a_) == HIGH) ? 2 : 0;
    const uint8_t b = (digitalRead(pin_b_) == HIGH) ? 1 : 0;
    last_state_ = a | b;
}

void Encoder::handleHostEdge() {
    const uint8_t a = (digitalRead(pin_a_) == HIGH) ? 2 : 0;
    const uint8_t b = (digitalRead(pin_b_) == HIGH) ? 1 : 0;
    const uint8_t curr = a | b;
    count_sim_ += kStep[(last_state_ << 2) | curr];
    last_state_ = curr;
}

int64_t Encoder::getCount() const {
    return count_sim_;
}

void Encoder::reset() {
    count_sim_ = 0;
}

#endif
