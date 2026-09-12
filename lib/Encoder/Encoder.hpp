#pragma once
#include <Arduino.h>

namespace mirs2605 {

// 単一の直交エンコーダを表すクラス。
// A相CHANGE検出（X2：2逓倍、1024PPR→2048カウント/回転）。
//
// このクラスは純粋に数えるだけ。正逆の意味づけは持たない。
// 取付方向に由来する符号の吸収はDiffDrive側の仕事。
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 2;  // X2：2逓倍

    Encoder(uint8_t pin_a, uint8_t pin_b);

    void begin();               // pinMode + attachInterrupt(A相CHANGE)
    int32_t getCount() const;   // スナップショット取得
    void reset();
    uint32_t countsPerRev() const;  // PULSES_PER_REV * MULTIPLIER (=2048)

private:
    static void IRAM_ATTR isrA(void *arg);
    void handleA();

    uint8_t pin_a_, pin_b_;
    volatile int32_t count_;
};


}  // namespace mirs2605