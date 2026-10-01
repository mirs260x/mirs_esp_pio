#pragma once
#include <Arduino.h>
#if defined(ESP_PLATFORM)
#include "driver/pulse_cnt.h"
#endif

/** @brief 単一の直交エンコーダ。X4換算（1024PPR→4096カウント/車輪1回転、実測）。
 *  @details ESP32実機ではPCNTハードで計数、ホストテスト(native)ではソフト計数。
 *  純粋計数のみで正逆の意味づけは持たない。取付方向の符号吸収はDiffDrive側の仕事。
 *  @note 旧X2=2048では距離が2倍に出るため4096に修正。 */
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 4;  // X4換算

    Encoder(uint8_t pin_a, uint8_t pin_b);

    void begin();               // PCNT unit+channel生成・start (hostではpinMode+attachInterrupt)
    int32_t getCount() const;   // スナップショット取得
    void reset();
    uint32_t countsPerRev() const;  // PULSES_PER_REV * MULTIPLIER (=4096)

private:
#if defined(ESP_PLATFORM)
    uint8_t pin_a_, pin_b_;
    mutable pcnt_unit_handle_t unit_ = nullptr;
    pcnt_channel_handle_t chan_ = nullptr;
    mutable int32_t last_ok_ = 0;  // 読取失敗時は前回値を返す（0誤読による跳躍防止）
    bool initialized_ = false;
#else
    static void IRAM_ATTR isrA(void *arg);
    void handleA();

    uint8_t pin_a_, pin_b_;
    volatile int32_t count_;
#endif
};
