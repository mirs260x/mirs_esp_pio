#pragma once
#include <Arduino.h>

// MR-8 (近藤科学) 等、標準的なRCサーボPWM出力の受信機を読み取るクラス
// 各チャンネルは周期20ms、パルス幅約1000-2000usのPWM信号を出力する前提
class RcReceiver {
public:
    static constexpr uint8_t MAX_CH = 4;

    void begin(const uint8_t ch_pins[], uint8_t num_channels);

    // 指定チャンネルの現在のパルス幅 [us]
    uint16_t getPulseWidth(uint8_t channel);

    // 直近 timeout_ms 以内に信号更新があったか (フェイルセーフ判定用)
    bool isSignalValid(uint8_t channel, uint32_t timeout_ms);

    // パルス幅[us] を -1.0〜1.0 に正規化 (デッドゾーン付き)
    static float pulseToNormalized(uint16_t pulse_us);

private:
    static volatile uint32_t _rise_time[MAX_CH];
    static volatile uint16_t _pulse_width[MAX_CH];
    static volatile uint32_t _last_update[MAX_CH];
    static uint8_t _pins[MAX_CH];
    static uint8_t _num_ch;

    static void IRAM_ATTR isr(void *arg);
};
