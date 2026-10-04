#include "Encoder.hpp"

namespace {

// 最短エッジ間隔 [us]。これ未満の連続エッジはノイズとして無視する（状態も進めない）。
// 最高速0.8m/s時のエッジ間隔は約37us以上のため実信号は通る。
constexpr uint32_t kMinEdgeIntervalUs = 10;

// 直交状態遷移表。index=(prev<<2)|curr、bit1=A・bit0=B。
// +1方向: 00→01→11→10→00。配線逆転の符号はDiffDriveのreverse指定で吸収する。
constexpr int8_t kStep[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

}  // namespace

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a_(pin_a)
    , pin_b_(pin_b)
{
}

void Encoder::begin() {
    if (initialized_) {
        return;
    }
    // フローティングによるモータノイズ誤計数を防ぐ。外部プルアップがあっても害はない。
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
    noInterrupts();
    last_state_ = readState();
    last_edge_us_ = micros();
    interrupts();
    attachInterruptArg(digitalPinToInterrupt(pin_a_), isr, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pin_b_), isr, this, CHANGE);
    initialized_ = true;
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
    last_state_ = readState();
    last_edge_us_ = micros();
    interrupts();
}

uint32_t Encoder::countsPerRev() const {
    return PULSES_PER_REV * MULTIPLIER;
}

void IRAM_ATTR Encoder::isr(void *arg) {
    static_cast<Encoder *>(arg)->handleEdge();
}

uint8_t Encoder::readState() const {
    const uint8_t a = (digitalRead(pin_a_) == HIGH) ? 2 : 0;
    const uint8_t b = (digitalRead(pin_b_) == HIGH) ? 1 : 0;
    return static_cast<uint8_t>(a | b);
}

void Encoder::handleEdge() {
    const uint32_t now = micros();
    if (now - last_edge_us_ < kMinEdgeIntervalUs) {
        return;
    }
    last_edge_us_ = now;
    const uint8_t curr = readState();
    const uint8_t prev = last_state_;
    last_state_ = curr;
    count_ += kStep[(prev << 2) | curr];
}
