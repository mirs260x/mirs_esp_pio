#pragma once
#include <Arduino.h>

/** @brief RCサーボPWM受信機リーダ (MR-8等、890〜2100us、中立1496us前提)。
 *  @details GPIO割込み両エッジでパルス幅を計測し毎周期更新する。 */
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
    static volatile uint16_t _pulse_width[MAX_CH];
    static volatile uint32_t _last_update[MAX_CH];
    static volatile uint32_t _rise_time[MAX_CH];
    static uint8_t _pins[MAX_CH];
    static uint8_t _num_ch;

    static void IRAM_ATTR isr(void *arg);
};
