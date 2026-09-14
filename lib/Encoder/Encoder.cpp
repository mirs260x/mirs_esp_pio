#include "Encoder.hpp"

#if defined(ESP_PLATFORM)

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a)
    , pin_b_(pin_b)
{
}

void Encoder::begin() {
    if (initialized_) {
        return;
    }
    // 16bitカウンタの折返しをドライバ側で累積させる
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit = -32768;
    unit_cfg.high_limit = 32767;
    unit_cfg.flags.accum_count = 1;
    if (pcnt_new_unit(&unit_cfg, &unit_) != ESP_OK) {
        return;
    }
    // チャタリング・ノイズ除去 (1us未満を無視)
    pcnt_glitch_filter_config_t filt_cfg = {};
    filt_cfg.max_glitch_ns = 1000;
    (void)pcnt_unit_set_glitch_filter(unit_, &filt_cfg);

    // X2: A相の両エッジで計数、方向はB相レベルで判定。
    // 計数式: 立上り+B高→+ / 立上り+B低→- / 立下り+B高→- / 立下り+B低→+
    pcnt_chan_config_t chan_cfg = {};
    chan_cfg.edge_gpio_num = pin_a_;
    chan_cfg.level_gpio_num = pin_b_;
    if (pcnt_new_channel(unit_, &chan_cfg, &chan_) != ESP_OK) {
        return;
    }
    (void)pcnt_channel_set_edge_action(chan_,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    (void)pcnt_channel_set_level_action(chan_,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    if (pcnt_unit_enable(unit_) != ESP_OK) {
        return;
    }
    (void)pcnt_unit_clear_count(unit_);
    if (pcnt_unit_start(unit_) != ESP_OK) {
        return;
    }
    initialized_ = true;
}

int32_t Encoder::getCount() const {
    if (unit_ == nullptr) {
        return 0;
    }
    int value = 0;
    if (pcnt_unit_get_count(unit_, &value) != ESP_OK) {
        return 0;
    }
    return static_cast<int32_t>(value);
}

void Encoder::reset() {
    if (unit_ != nullptr) {
        (void)pcnt_unit_clear_count(unit_);
    }
}

uint32_t Encoder::countsPerRev() const {
    return PULSES_PER_REV * MULTIPLIER;
}

#else

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a)
    , pin_b_(pin_b)
    , count_(0)
{
}

void Encoder::begin() {
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(pin_a_), isrA, this, CHANGE);
}

int32_t Encoder::getCount() const {
    noInterrupts();
    int32_t snap = count_;
    interrupts();
    return snap;
}

void Encoder::reset() {
    noInterrupts();
    count_ = 0;
    interrupts();
}

uint32_t Encoder::countsPerRev() const {
    return PULSES_PER_REV * MULTIPLIER;
}

void IRAM_ATTR Encoder::isrA(void *arg) {
    static_cast<Encoder *>(arg)->handleA();
}

void Encoder::handleA() {
    // A変化後のAとBを比較する。
    count_ += (digitalRead(pin_a_) == digitalRead(pin_b_)) ? +1 : -1;
}

#endif
