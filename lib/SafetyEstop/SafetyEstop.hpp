#pragma once
#include <Arduino.h>

// E-Stopの状態はMotorDriver.cppからも直接参照されるグローバルフラグ
// (通信経路やmain loopの詰まりに影響されず、確実にモータを止めるため)
extern volatile bool g_estop_active;

class SafetyEstop {
public:
    void begin(uint8_t pin);

    // RC信号ロストなど、割り込み以外の要因からもトリガーできるようにする
    static void trigger();

    // 物理スイッチが解除されている、かつ明示的なリセット操作があった場合のみ復帰
    static void reset();

private:
    static void IRAM_ATTR isr();
    static uint8_t _pin;
};
