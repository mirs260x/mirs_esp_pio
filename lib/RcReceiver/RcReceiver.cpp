#include "RcReceiver.hpp"
#include "config.h"

volatile uint32_t RcReceiver::_rise_time[MAX_CH];
volatile uint16_t RcReceiver::_pulse_width[MAX_CH];
volatile uint32_t RcReceiver::_last_update[MAX_CH];
uint8_t RcReceiver::_pins[MAX_CH];
uint8_t RcReceiver::_num_ch = 0;

void IRAM_ATTR RcReceiver::isr(void *arg) {
    uint8_t ch = (uint32_t)arg;
    if (digitalRead(_pins[ch]) == HIGH) {
        _rise_time[ch] = micros();
    } else {
        _pulse_width[ch] = (uint16_t)(micros() - _rise_time[ch]);
        _last_update[ch] = millis();
    }
}

void RcReceiver::begin(const uint8_t ch_pins[], uint8_t num_channels) {
    _num_ch = num_channels;
    for (uint8_t i = 0; i < num_channels && i < MAX_CH; i++) {
        _pins[i] = ch_pins[i];
        pinMode(_pins[i], INPUT);
        attachInterruptArg(digitalPinToInterrupt(_pins[i]), isr, (void *)(uint32_t)i, CHANGE);
    }
}

uint16_t RcReceiver::getPulseWidth(uint8_t channel) {
    noInterrupts();
    uint16_t val = _pulse_width[channel];
    interrupts();
    return val;
}

bool RcReceiver::isSignalValid(uint8_t channel, uint32_t timeout_ms) {
    noInterrupts();
    uint32_t last = _last_update[channel];
    interrupts();
    return (millis() - last) < timeout_ms;
}

float RcReceiver::pulseToNormalized(uint16_t pulse_us) {
    if (abs((int)pulse_us - RC_PULSE_MID) < RC_PULSE_DEADZONE) {
        return 0.0f;
    }
    float norm = ((float)pulse_us - RC_PULSE_MID) / ((RC_PULSE_MAX - RC_PULSE_MIN) / 2.0f);
    return constrain(norm, -1.0f, 1.0f);
}
