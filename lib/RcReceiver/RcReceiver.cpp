#include "RcReceiver.hpp"
#include "hardware_config.hpp"

volatile uint16_t RcReceiver::_pulse_width[MAX_CH];
volatile uint32_t RcReceiver::_last_update[MAX_CH];
volatile uint32_t RcReceiver::_rise_time[MAX_CH];
uint8_t RcReceiver::_pins[MAX_CH];
uint8_t RcReceiver::_num_ch = 0;

void IRAM_ATTR RcReceiver::isr(void *arg) {
    uint8_t ch = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(arg));
    if (ch >= _num_ch) {
        return;
    }
    if (digitalRead(_pins[ch]) == HIGH) {
        _rise_time[ch] = micros();
    } else {
        uint32_t width = micros() - _rise_time[ch];
        // 有効なRCサーボパルス (RC_PULSE_MIN〜RC_PULSE_MAX) のみを受け入れる（ノイズ除去）
        if (width >= RC_PULSE_MIN && width <= RC_PULSE_MAX) {
            _pulse_width[ch] = static_cast<uint16_t>(width);
            _last_update[ch] = millis();
        }
    }
}

void RcReceiver::begin(const uint8_t ch_pins[], uint8_t num_channels) {
    _num_ch = (num_channels < MAX_CH) ? num_channels : MAX_CH;
    for (uint8_t i = 0; i < _num_ch; i++) {
        _pins[i] = ch_pins[i];
        // 未接続時のフローティング防止。受信機接続時は受信機がラインを駆動する
        pinMode(_pins[i], INPUT_PULLDOWN);
        attachInterruptArg(digitalPinToInterrupt(_pins[i]), isr,
                           reinterpret_cast<void *>(static_cast<uintptr_t>(i)), CHANGE);
    }
}

uint16_t RcReceiver::getPulseWidth(uint8_t channel) {
    if (channel >= _num_ch) {
        return 0;
    }
    noInterrupts();
    uint16_t val = _pulse_width[channel];
    interrupts();
    return val;
}

bool RcReceiver::isSignalValid(uint8_t channel, uint32_t timeout_ms) {
    if (channel >= _num_ch) {
        return false;
    }
    noInterrupts();
    uint32_t last = _last_update[channel];
    interrupts();
    return (last != 0) && ((millis() - last) < timeout_ms);
}

float RcReceiver::pulseToNormalized(uint16_t pulse_us) {
    // 範囲外のパルス幅は 0 (停止) として処理
    // スティック: 890〜2100で倒れ度合い、中立1496。SW: 1496→1995でON
    if (pulse_us < RC_PULSE_MIN || pulse_us > RC_PULSE_MAX) {
        return 0.0f;
    }
    if (abs(static_cast<int>(pulse_us) - RC_PULSE_MID) < RC_PULSE_DEADZONE) {
        return 0.0f;
    }
    float norm = (static_cast<float>(pulse_us) - RC_PULSE_MID) / ((RC_PULSE_MAX - RC_PULSE_MIN) / 2.0f);
    return constrain(norm, -1.0f, 1.0f);
}
