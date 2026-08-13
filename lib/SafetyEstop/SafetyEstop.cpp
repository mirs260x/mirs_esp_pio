#include "SafetyEstop.hpp"

volatile bool g_estop_active = false;
uint8_t SafetyEstop::_pin = 0;

void IRAM_ATTR SafetyEstop::isr() {
    g_estop_active = true;
}

void SafetyEstop::begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, INPUT_PULLUP);
    // FALLING: スイッチがGNDに落ちたらトリガーする配線を想定。実機の結線に合わせて変更すること
    attachInterrupt(digitalPinToInterrupt(_pin), isr, FALLING);

    // 起動直後にスイッチが既に押されている(閉)状態なら安全側で開始
    if (digitalRead(_pin) == LOW) {
        g_estop_active = true;
    }
}

void SafetyEstop::trigger() {
    g_estop_active = true;
}

void SafetyEstop::reset() {
    // 物理スイッチが解除(HIGH)されている場合のみ復帰を許可
    if (digitalRead(_pin) == HIGH) {
        g_estop_active = false;
    }
}
