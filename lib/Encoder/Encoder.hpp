#pragma once
#include <Arduino.h>

// 単一の直交エンコーダを表すクラス。
// A相CHANGE検出（X2：2逓倍、1024PPR→2048カウント/回転）。
//
// 正方向の定義はハードウェア基準：コンストラクタに渡すA/Bピンの順序で決まる。
// ミラー実装の左輪は Encoder(PIN_ENC_B_L, PIN_ENC_A_L) のように逆順で渡す。
// ソフトウェア側の反転フラグは持たない。
// ハード変更（取付反転等）があればピン指定順の変更だけで対応できる。
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
