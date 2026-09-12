# 全体設計書（FWマルチスレッド化）

## 1. 目的

- プロポ系とROS2系を分離し、**プロポ単独で走行可能**にする
- ROS2接続中は制御モードに関わらずテレメトリを出し続け、**プロポ操縦でのマッピング**を可能にする
- **プロポによる制御権奪取**（モードSW）を維持する
- FreeRTOSタスク化＋ノンブロッキング化で応答性と保守性を上げる

## 2. 動作モード

| モード | 速度指令源 | モータ出力 | ROS2の要否 |
|---|---|---|---|
| MANUAL（プロポ） | RCスティック → 開ループ直結 | 必須で動作 | 不要。接続中はテレメトリ継続 |
| ROS2（自律） | `/cmd_vel` → PID閉ループ | PID経由 | 要 |

- モード切替は現行どおりモードSWの立ち上がりエッジでトグル（維持）
- ROS2モード中のプロポ操作（SW）で即MANUALに奪取できる（維持）
- MANUAL中のROS2指令は無視するが、`/encoder`・オドメトリ・IMU等の発行は継続する（マッピング用途）

## 3. タスク構成案

| タスク | Core | 周期/優先度 | 役割 | ROS依存 |
|---|---|---|---|---|
| `control` | 0 | 15ms・高 | RC読取→調停→PID/直結→モータ出力 | なし（単独で完結） |
| `ros_comm` | 1 | イベント駆動・中 | agent接続、`/cmd_vel`→キュー投入、テレメトリ発行、`/params` 反映 | あり（不在でも他タスクは継続） |
| `sensor` | 1 | 50ms・低 | IMU・電圧ポーリング（I2Cはタイムアウト付き） | なし |

- `control` はROSの生死を知らない。`ros_setup()` のブロッキング（`delay(2000)` 等）は `ros_comm` 内に隔離し、agent未接続でも制御に影響させない
- PWM出力（`ledcWrite`）は元々ノンブロッキング。`analogRead` 平均化ループ・I2C読取は `sensor` タスクに移し、制御周期を乱さない

## 4. データフロー

```
[RC受信機] ──→ ┌──────────────┐
               │ control task │ ──→ MotorController ──→ MD10C
/cmd_vel ──→ [queue] ─→ │ (調停:MANUAL/ROS2) │
               └──────┬───────┘
[Encoder×2] ──→ DifferentialDrive ─┬─→ Odometry ─→ 共有( mutex ) ─→ /encoder・/odom系
                                   └→ PID feedback（ROS2モード時のみ使用）
[sensor task] ──→ 共有(mutex): IMU・電圧 ─→ /imu・/vlt
/params ──→ 共有(mutex): PID・車体パラメータ ─→ control/sensor両参照
```

- タスク間はキュー（速度指令）とmutex付き共有（オドメトリ・パラメータ）で受渡す
- ISR↔タスク間は現行どおりcritical/noInterrupts継続
- `Encoder`・`DifferentialDrive`・`Odometry` の3層はタスク非依存のためそのまま `control` 側で使う

## 5. フェイルセーフ

| 条件 | 動作 |
|---|---|
| ROS2指令途絶（watchdog） | ROS2モード時は速度指令ゼロ（維持） |
| RC信号喪失 | MANUAL時は停止。ROS2モード時はSW検出不能のため奪取不可、watchdogに委ねる |
| agent未接続・切断 | `ros_comm` のみ再接続待ち。`control` は継続 |
| 非常停止・電圧カットオフ | 回路・配線実装後に有効化（現状無効のまま） |

## 6. PIDの扱い

- MANUAL：開ループ直結（フィードバックなし）
- ROS2：PID閉ループ（エンコーダ速度フィードバックあり）
- エンコーダ自体は両モードで稼働し、`/encoder` テレメトリとROS側オドメトリに供給する

## 7. ファイル構成（目標）

```
src/
  main.cpp            # 薄くする：初期化＋タスク生成のみ（現行control_loop等は移管予定）
  tasks/
    control_task.*    # 骨格のみ（TODO）
    ros_task.*        # 骨格のみ（TODO）
    sensor_task.*     # 骨格のみ（TODO）
lib/
  SystemContext/      # 新設：タスク間共有（キュー・mutex）
  Encoder/ DifferentialDrive/ Odometry/  # 済
  （以下は恩赦：現状維持）
```

### 恩赦リスト（意図的に対処を保留する競合・重複）

| 対象 | 状態 | 対処時期 |
|---|---|---|
| 旧 `MotorController`（ペア＋DIR論理） | 新 `MotorDriver`＋`DifferentialMotors` に置換済みのため superseded。`main.cpp` が参照中のため残置 | `main.cpp` 移管時に削除 |
| `VelocityCalculator` | 存続。将来 `DifferentialDrive` 吸収候補 | 組み込み時に判断 |
| `RobotController` 改名（調停者として不正確） | 改名せず存続 | 参照整理後に判断 |
| `SafetyEstop` 無効・電圧カットオフ無効 | 無効のまま存続 | 回路・配線実装時 |

※ 旧 `MotorDriver`・`src/CMakeLists.txt` は削除済み（2026-09-12）。`SafetyEstop.hpp` の陳腐コメントも修正済み。

## 8. 未確定事項

1. タスク優先度・コア割当の確定（上表は案）
2. キュー深度・mutex範囲の確定
3. ROS不在時の初回起動シーケンス（agent待ちの上限・リトライ間隔）
4. RC信号喪失時のROS2モード継続可否（現行はRC必須＝停止する。緩和するか）
5. 電圧カットオフ・非常停止の有効化時期
