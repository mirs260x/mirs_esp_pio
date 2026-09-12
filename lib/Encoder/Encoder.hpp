#pragma once
#include <Arduino.h>

// 単一の直交エンコーダを表すクラス。
// 検出方式はA相CHANGEのみ（X2：2逓倍、1024PPR→2048カウント/回転）。
// 差動二輪はこのオブジェクト2つ（DifferentialDrive）で表現する。
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 2;  // X2：2逓倍

    // reverse=trueで正転・逆転の符号を反転する。
    // 機体配線に由来する左右の非対称（現行main.cppのL/Rで符号が逆）は
    // こちらで吸収し、カウント式自体は左右共通にする。
    explicit Encoder(uint8_t pin_a, uint8_t pin_b, bool reverse = false);

    void begin();               // pinMode + attachInterrupt(A相CHANGE)
    int32_t getCount() const;   // スナップショット取得
    void reset();
    uint32_t countsPerRev() const;  // PULSES_PER_REV * MULTIPLIER (=2048)

private:
    static void IRAM_ATTR isrA(void *arg);
    void handleA();

    uint8_t pin_a_, pin_b_;
    bool reverse_;
    volatile int32_t count_;
};
