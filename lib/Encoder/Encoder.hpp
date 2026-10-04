#pragma once
#include <Arduino.h>

/** @brief 単一の直交エンコーダ。X4換算（1024PPR→4096カウント/車輪1回転）。
 *  @details 外部ライブラリ不使用。GPIO割込み（A/B両相CHANGE）で自前計数する。
 *  PCNT等のHWカウンタは使わない（方針）。純粋計数のみで正逆の意味づけは持たない。
 *  取付方向の符号吸収はDiffDrive側の仕事。
 *  チャタリング対策として10us未満の連続エッジを無視する。
 *  最高速0.8m/s時のエッジ間隔は約37us以上のため実信号は通る。
 *  @note ホストテストと同一実装のため、計数ロジックは `pio test -e native` で検証できる。
 */
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 4;  // X4換算（A/B両相の両エッジ）

    Encoder(uint8_t pin_a, uint8_t pin_b);

    void begin();               // プルアップ＋両相CHANGE割込み登録（多重beginは無視）
    int32_t getCount() const;   // スナップショット取得
    void reset();               // 計数と基準状態を初期化
    uint32_t countsPerRev() const;  // PULSES_PER_REV * MULTIPLIER (=4096)

private:
    static void IRAM_ATTR isr(void *arg);
    void handleEdge();
    uint8_t readState() const;  // bit1=A・bit0=B の2bit状態

    uint8_t pin_a_, pin_b_;
    volatile int32_t count_ = 0;
    volatile uint8_t last_state_ = 0;
    volatile uint32_t last_edge_us_ = 0;
    bool initialized_ = false;
};
