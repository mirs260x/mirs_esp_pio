# 0005: エンコーダ計数のPCNT排除とGPIO自前計数化

- Status: 採用
- Date: 2026-10-04

## 背景

- `Encoder` がESP-IDFのPCNTドライバ（`driver/pulse_cnt.h`）に依存していた。外部ライブラリを使わない方針と衝突する。
- ホストテストのモックはA相のみのX2相当で、実機のX4換算を検証できなかった（乖離）。

## 決定

- PCNTを使わず、GPIO割込み（A/B両相CHANGE）＋直交状態遷移表で自前計数する（真のX4）。
- チャタリング対策は10us未満の連続エッジ無視で行う（最高速時の実信号は約37us間隔）。
- プルアップはArduinoの `INPUT_PULLUP` で行う（ESP-IDFのGPIO API不使用）。
- ホストと同一実装にし、`pio test -e native` で計数ロジック自体を検証する。

## 結果

- `pulse_cnt.h` 依存を除去。`begin`/`getCount`/`reset` のIFは不変のため `DiffDrive`・`control_task` の変更なし。
- 代償：PCNTのHWグリッチフィルタ・ドライバ側累積がなくなる。ノイズ耐性は10usガード＋既存の単発棄却（±2048/周期）で担保する。実機での誤計数確認はTODOの実機評価で行う。
