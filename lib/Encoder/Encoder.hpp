#pragma once
#include <Arduino.h>
#if defined(ESP_PLATFORM)
#include "driver/pulse_cnt.h"
#endif

/** @brief 単一の直交エンコーダ。X2 (2逓倍、1024PPR→2048カウント/回転)。
 *  @details ESP32実機ではPCNTハードで計数、ホストテスト(native)ではソフト計数。
 *  純粋計数のみで正逆の意味づけは持たない。取付方向の符号吸収はDiffDrive側の仕事。 */
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 2;  // X2：2逓倍

    Encoder(uint8_t pin_a, uint8_t pin_b);

    void begin();               // PCNT unit+channel生成・start (hostではpinMode+attachInterrupt)
    int32_t getCount() const;   // スナップショット取得
    void reset();
    uint32_t countsPerRev() const;  // PULSES_PER_REV * MULTIPLIER (=2048)

private:
#if defined(ESP_PLATFORM)
    uint8_t pin_a_, pin_b_;
    mutable pcnt_unit_handle_t unit_ = nullptr;
    pcnt_channel_handle_t chan_ = nullptr;
    bool initialized_ = false;
#else
    static void IRAM_ATTR isrA(void *arg);
    void handleA();

    uint8_t pin_a_, pin_b_;
    volatile int32_t count_;
#endif
};
