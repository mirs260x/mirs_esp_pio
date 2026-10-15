#pragma once
#include <stdint.h>
#include <Arduino.h>

#ifdef ARDUINO
#include "driver/pulse_cnt.h"
#endif

/**
 * @brief AMT103-V 直交エンコーダ（PCNT ハードウェア4逓倍計数）。
 * @details
 *  - デフォルト 2048 PPR × 4逓倍 = 8192 カウント/回転。
 *  - ESP32 ハードウェア PCNT を使用して CPU 負荷ゼロで計数。
 *  - PCNTドライバの累積機能で16-bitカウンタの折返しを吸収し、
 *    連続カウント値として取得します。
 */
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 2048;
    static constexpr uint32_t MULTIPLIER = 4; // X4 4逓倍
    static constexpr int16_t PCNT_LIMIT = 20000;

#ifdef ARDUINO
    Encoder(uint8_t pin_a, uint8_t pin_b);
#else
    Encoder(uint8_t pin_a, uint8_t pin_b, int unit = 0);
#endif

    void begin();
    int64_t getCount() const;
    void reset();
    uint32_t countsPerRev() const { return PULSES_PER_REV * MULTIPLIER; }

#ifndef ARDUINO
    // テスト・シミュレーション用
    void handleHostEdge();
    void simAddCount(int64_t delta) { count_sim_ += delta; }
#endif

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
#ifdef ARDUINO
    pcnt_unit_handle_t unit_ = nullptr;
    pcnt_channel_handle_t channel_a_ = nullptr;
    pcnt_channel_handle_t channel_b_ = nullptr;
    mutable int32_t last_pcnt_count_ = 0;
    mutable int64_t extended_count_ = 0;
#else
    int unit_;
    int64_t count_sim_ = 0;
    uint8_t last_state_ = 0;
#endif
    bool initialized_ = false;
};
