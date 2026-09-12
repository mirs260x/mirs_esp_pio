# CuGo V3 + ESP32 micro-ROS ファームウェア

CuGo V3クローラ、MD10Cモータドライバ、プロポ受信機、ESP32 (Arduino framework)、
PlatformIO、micro-ROS (ROS 2 Jazzy, Serial transport) を使用した足回り制御ファームウェア。

プロポ単独で走行でき、ROS 2接続時は自律走行とテレメトリ配信を行う。

## 目次

- [使い方](#使い方)
  - [セットアップ手順](#セットアップ手順)
  - [micro-ROS Agent](#micro-ros-agent)
  - [Topics](#topics)
  - [動作モード](#動作モード)
- [仕様](#仕様)
  - [アーキテクチャ](#アーキテクチャ)
  - [層対応表](#層対応表)
  - [エンコーダ・オドメトリ仕様](#エンコーダオドメトリ仕様)
  - [モータ仕様](#モータ仕様)
  - [ライブラリ一覧](#ライブラリ一覧)
  - [フェイルセーフ](#フェイルセーフ)
  - [スレッドセーフ方針](#スレッドセーフ方針)
- [実機投入前に必ず確認・調整すること](#実機投入前に必ず確認調整すること)
- [安全設計メモ](#安全設計メモ)

## 使い方

### セットアップ手順

```bash
# ビルド
pio run -e esp32dev

# 書き込み
pio run -e esp32dev --target upload

# 単体テスト（ホスト実行・実機不要）
pio test -e native

# シリアルモニタ (micro-ROSと同時使用不可、デバッグ時のみ)
pio device monitor
```

### micro-ROS Agent（PC/ROS 2側）

```bash
# Docker利用の場合
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev [ESP32のシリアルポート] -v6

# ネイティブROS 2 Jazzy環境の場合
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200
```

### Topics

| Topic | 型 | 方向 | 周期 | 説明 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | — | ROS2モード時の速度指令 |
| `/params` | `mirs_msgs/BasicParam` | Subscribe | — | 車輪・PIDパラメータ |
| `/encoder` | `std_msgs/Int32MultiArray` | Publish | 約17Hz | `[左, 右]` エンコーダカウント |
| `/vel` | `std_msgs/Float64MultiArray` | Publish | 約17Hz | `[左, 右]` 車輪速度 [m/s] |
| `/vlt` | `std_msgs/Float64MultiArray` | Publish | 約17Hz | `[v1, v2]` バッテリー電圧 [V] |
| `/rc_debug` | `std_msgs/Float64MultiArray` | Publish | 約17Hz | RC脈幅・指令・モード |
| `/imu/data_raw` | `sensor_msgs/Imu` | Publish | 約67Hz | 加速度・角速度 |
| `/imu/mag` | `sensor_msgs/MagneticField` | Publish | 約67Hz | 地磁気 |

### 動作モード

| モード | 速度指令源 | ROS 2 |
|---|---|---|
| MANUAL（プロポ） | スティック → 開ループ直結 | 不要。接続中はテレメトリ継続（マッピング可） |
| ROS2（自律） | `/cmd_vel` → PID閉ループ | 要 |

- 起動直後は `MODE_MANUAL` がデフォルト
- モードSWの立ち上がりエッジでトグル。ROS2走行中もプロポ操作で制御権を奪取できる
- 受信信号が100ms以上途切れた場合は速度指令をゼロにする
- チャンネル割当やピンは `include/hardware_config.hpp` で変更する

## 仕様

### アーキテクチャ

FreeRTOSの3タスク構成。`src/main.cpp` は初期化＋タスク生成のみ。

| タスク | Core | 周期/優先度 | 役割 | ROS依存 |
|---|---|---|---|---|
| `control` | 0 | 15ms・10 | RC読取→調停→PID/直結→モータ出力 | なし（単独で完結） |
| `ros_comm` | 1 | spin・5 | agent接続、`/cmd_vel`→キュー投入、テレメトリ発行、`/params` 反映 | あり（不在でも他タスクは継続） |
| `sensor` | 1 | 15ms・3 | IMU・電圧ポーリング（I2Cはタイムアウト付き） | なし |

データフロー：

```
[RC受信機] ──→ ┌──────────────┐
               │ control task │ ──→ DiffMotors ──→ MD10C
/cmd_vel ──→ [queue] ─→ │ (RobotController調停) │
               └──────┬───────┘
[Encoder ISR] → counts → VelocityCalculator → PID → (ROS2時)
/encoder ← counts ┐
/vel ← 速度       ├─ SharedMotion (control→ros)
/imu・/vlt ← SharedSensor (sensor→ros)。IMU 66Hz維持、電圧4分周
/params ──→ SharedParams (ros→control/sensor)
```

- HWタイマ＋フラグ方式は廃止し、`vTaskDelayUntil` による15ms周期に統一した
- モータ出力段は新層（`MotorDriver`＋`DiffMotors`）に切替済み。エンコーダ側新層（`Encoder`・`DiffDrive`・`Odometry`）の `control` 組込は `TODO.md` のP0項目として残存

### 層対応表

ディレクトリは機能別のまま、層はこの表で管理する。**下の層は上の層を知ってはならない**（逆流禁止）。

```
計算層:       Odometry, VelocityCalculator, PIDController
  ↓ 参照のみ（逆流禁止）
IF層:         DiffDrive, DiffMotors
  ↓ 参照のみ（逆流禁止）
ハード層:     Encoder, MotorDriver, BMX055, RcReceiver, VoltageSensor
横断:         SystemContext（層を持たず、タスク間共有専用）
              RobotController（調停者。tasks側から使う）
```

### エンコーダ・オドメトリ仕様

- 検出方式はA相CHANGEのみ（X2：2逓倍、1024PPR→2048カウント/回転）
- 方向判定式は `A==B → +1`、不一致 → `-1`。B相は読むだけでエッジ検出しない
- `Encoder` 自体は正逆の意味づけを持たず純粋計数。左右の非対称（ミラー実装）は `DiffDrive` のreverse指定（左`true`）で吸収する。ハード変更時はその指定だけ変え、`Odometry` には不可視
- 運動学（ROS側と同一式）：`d=(l+r)/2`、`dtheta=(r-l)/wheel_base`
  - **中点法**：`mid=theta+dtheta/2` で並進積分（円弧誤差低減）
  - `theta` は `[-PI, PI]` に正規化（長時間運転のfloat精度劣化防止）
- 確定パラメータ：`wheel_base = 0.39`（床面テープ基準で旋回360°一致確認済み）。路面・機体が変わったら再較正すること。ROS側 `config.yaml` と同値に保つ

### モータ仕様

```
PIDController（計算：速度→duty。既存流用）
      ↓ duty [-255, +255]
DiffMotors（ペアIF：極性吸収＋左右分配）
      ↓ duty（極性補正済み）
MotorDriver ×2（デバイス：MD10C単chのPWM+DIR出力）
```

- duty域は-255..+255のint（PID出力制限±255・8bit ledcと一致）
- 極性既定値は現行と一致（左 正=DIR HIGH、右 正=DIR LOW）。ハード変更時はreverse指定だけ変える
- 非常停止の判断は持たない（制御層の仕事）

### ライブラリ一覧

| ライブラリ | 役割 | 状態 |
|---|---|---|
| `Encoder` | 直交エンコーダ | 新経路。`control` 組込待ち |
| `DiffDrive` | IF層：カウント差分→移動距離（速度計算なし） | 新経路。`control` 組込待ち |
| `Odometry` | 計算層：移動距離→自己位置・姿勢 | 新経路。`control` 組込待ち |
| `MotorDriver` | デバイス：MD10C単ch出力 | 使用中（`control` タスクの出力段） |
| `DiffMotors` | IF層：モータペア＋極性吸収 | 使用中（`control` タスクの出力段） |
| `SystemContext` | タスク間共有（キュー・mutex） | 使用中 |
| `PIDController` | 計算層：速度PID（ROS2モード用） | 使用中 |
| `VelocityCalculator` | 計算層：カウント→車輪速度 | 使用中（`DiffDrive` への吸収はしない方針） |
| `RobotController` | MANUAL/ROS2調停・ウォッチドッグ | 使用中（改名は保留） |
| `RcReceiver` | プロポPWM読取 | 使用中 |
| `VoltageSensor` | バッテリー電圧監視（ADC） | 使用中 |
| `BMX055` | IMU | 使用中 |
| `SafetyEstop` | E-Stop | 無効（回路実装待ち） |

配置方針：`src/`＝アプリの配線（`main.cpp`・tasks）、`lib/`＝プロジェクト内コンポーネント置き場。層の所属は[層対応表](#層対応表)で管理する。

命名方針：汎用部品（`Encoder`・`MotorDriver`・`PIDController` 等）は共通orgのrepo置きとし、チーム名を付けない。`mirs2605` 冠はIMU・EKF実装・独自msgs等の2605固有に限定する。

### ライブラリ公開手順

`.gitignore` の逆として、`tools/allow/<Lib>.txt` に収録ファイルだけを列挙する。submodule/subtree不要。

```bash
# 対象libだけステージング（他は混入しない）
git add --pathspec-from-file=tools/allow/Encoder.txt
```

### フェイルセーフ

| 条件 | 動作 |
|---|---|
| ROS2指令途絶 | 1秒（`WATCHDOG_TIMEOUT`）で速度指令ゼロ |
| RC信号喪失 | MANUAL時は停止。ROS2モード時はSW検出不能のため奪取不可、watchdogに委ねる |
| agent未接続・切断 | `ros` タスクのみ再接続待ち。`control` は継続 |
| 非常停止・電圧カットオフ | 回路・配線実装前のため無効 |

### スレッドセーフ方針

- タスク間：キュー（速度指令）とmutex付き共有（パラメータ・テレメトリ）。1構造体に2 writerを置かない
- ISR↔タスク間：critical/noInterrupts継続（`Encoder::getCount` 等）
- 上記により旧来の速度指令・速度変数の競合は構造的に解消した

## 実機投入前に必ず確認・調整すること

- [ ] `hardware_config.hpp` のピン番号を実際の配線に合わせる
- [ ] `wheel_base`（既定0.39）は床面基準の旋回360°で較正済み。路面・機体が変わったら再較正する
- [ ] `VOLTAGE_DIVIDER_RATIO` を実際の分圧抵抗値から計算し直す
- [ ] `VOLTAGE_CUTOFF_THRESHOLD` をバッテリー仕様に合わせる（有効化は配線確認後）
- [ ] `RC_PULSE_MIN/MID/MAX` をプロポの実測パルス幅で校正する
- [ ] E-Stopスイッチの配線極性（`SafetyEstop::begin()` の `FALLING` 設定）を実機に合わせる
- [ ] `platformio.ini` の `platform` バージョンを大会前に固定（pinning）する

## 安全設計メモ

- RC受信ロスト（電波切れ）では速度指令がゼロになる
- ROS2モードでは `/cmd_vel` 途絶から1秒（`WATCHDOG_TIMEOUT`）で速度指令がゼロになる
- E-Stop・電圧カットオフは回路・配線実装前のため無効。`TODO.md` で管理する
