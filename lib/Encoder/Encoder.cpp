#include "Encoder.hpp"

namespace mirs2605 {

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a)
    , pin_b_(pin_b)
    , count_(0)
{
}

void Encoder::begin() {
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(pin_a_), isrA, this, CHANGE);
}

int32_t Encoder::getCount() const {
    noInterrupts();
    int32_t snap = count_;
    interrupts();
    return snap;
}

void Encoder::reset() {
    noInterrupts();
    count_ = 0;
    interrupts();
}

uint32_t Encoder::countsPerRev() const {
    return PULSES_PER_REV * MULTIPLIER;
}

void IRAM_ATTR Encoder::isrA(void *arg) {
    static_cast<Encoder *>(arg)->handleA();
}

void Encoder::handleA() {
    // A変化後のAとBを比較する。
    count_ += (digitalRead(pin_a_) == digitalRead(pin_b_)) ? +1 : -1;
}


}  // namespace mirs2605