# モータ設計書

エンコーダと同様の3層分離をモータ側にも適用する。

## 1. 3層構成（MotorDriver → DifferentialMotors → PIDController）

```
PIDController（計算：速度→duty。既存流用）
      ↓ duty [-255, +255]
DifferentialMotors（ペアIF：極性吸収＋左右分配）
      ↓ duty（極性補正済み）
MotorDriver ×2（デバイス：MD10C単chのPWM+DIR出力）
```

| 層 | 役割 | 正逆・極性 |
|---|---|---|
| `MotorDriver` | 単chのduty出力。`setDuty(-255..+255)`、正=DIR HIGH | 持たない |
| `DifferentialMotors` | ペア保持＋極性吸収。`setBoth/setLeft/setRight/stop` | `reverse` 指定（既定：右反転＝現行DIR論理と一致） |
| `PIDController` | 速度→duty計算（既存。変更なし） | — |

## 2. 決定事項

- duty域は **-255..+255のint**（PID出力制限±255・8bit ledcと一致）。旧 `MotorDriver` の-1.0〜+1.0 float域は廃止
- 極性既定値は現行 `MotorController` と一致（左 正=HIGH、右 正=LOW）。`main.cpp` 移管時に振る舞いが変わらない
- 非常停止の判断は持たない（制御層の仕事。現状無効のため影響なし）
- 旧 `MotorDriver`（単ch・-1..1域・estop内包）は削除済み。旧 `MotorController` は `main.cpp` 移管時に削除予定

## 3. 検証

- `pio test -e native` の `test_motor` 8件：duty正負・clamp・stop・極性・無極性
- `pio run -e esp32dev` 通過
