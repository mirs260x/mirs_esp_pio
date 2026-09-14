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
| `/odom` | `nav_msgs/Odometry` | Publish | 約67Hz | 推定位置・姿勢・速度（frame: `odom` → `base_link`） |

### 動作モード

| モード | 速度指令源 | ROS 2 |
|---|---|---|
| MANUAL（プロポ） | スティック → 開ループ直結 | 不要。接続中はテレメトリ継続（マッピング可） |
| ROS2（自律） | `/cmd_vel` → PID閉ループ | 要 |

- 起動直後は `MODE_MANUAL` がデフォルト
- モードSWの立ち上がりエッジでトグル。ROS2からのトグル先は必ずMANUALのため、ROS2走行中の押下は必ず制御権を奪取できる
- 受信信号が100ms以上途切れた場合は速度指令をゼロにする
- チャンネル割当やピンは `include/hardware_config.hpp` で変更する

## 仕様

### アーキテクチャ

FreeRTOSの3タスク構成。`src/main.cpp` は初期化＋タスク生成のみ。

| タスク | Core | 周期/優先度 | 役割 | ROS依存 |
|---|---|---|---|---|
| `control` | 0 | 15ms・10 | 指令受付→調停→状態推定→PID→モータ出力 | なし（単独で完結） |
| `ros_comm` | 1 | spin・5 | agent接続、`/cmd_vel`→mailbox投入、テレメトリ発行、`/params` 反映 | あり（不在でも他タスクは継続） |
| `sensor` | 1 | 15ms・3 | IMU・電圧ポーリング（I2Cはタイムアウト付き） | なし |

データフロー：

```
[RC受信機] ──→ ┌──────────────┐
               │ control task │ ──→ DiffMotors ──→ MD10C
/cmd_vel ──→ [mailbox] ─→ │ (RobotController調停) │
               └──────┬───────┘
[Encoder/PCNT] → counts → VelocityCalculator → PID
                 ↘ 距離変換 → OdometryCalculator → pose
/encoder ← counts ┐
/vel ← 速度       │
/odom ← pose+速度  ├─ SharedMotion (control→ros)
/imu・/vlt ← SharedSensor (sensor→ros)。IMU 66Hz維持、電圧4分周
/params ──→ SharedParams (ros→control/sensor)。範囲外・NaNは棄却
```

- HWタイマ＋フラグ方式は廃止し、`vTaskDelayUntil` による15ms周期に統一した
- 計算層（`VelocityCalculator`・`PIDController`・`OdometryCalculator`）は `control` タスク内で実行する。`OdometryCalculator` のposeは `SharedMotion`（`odom_x/y/theta`）に格納し `/odom` で配信する。距離変換は本来IF層（`DiffDrive`）の仕事だが配置未確定のため当面control内で直計算する
- ハード資源：GPIO割込み 3ch（RC 3ch分、CHANGE両エッジ計測）、PCNT 2unit（左右エンコーダ）。他用途との競合に注意

### 層対応表

ディレクトリは機能別のまま、層はこの表で管理する。**下の層は上の層を知ってはならない**（逆流禁止）。

```
計算層:       OdometryCalculator, VelocityCalculator, PIDController
  ↓ 参照のみ（逆流禁止）
IF層:         DiffDrive, DiffMotors
  ↓ 参照のみ（逆流禁止）
ハード層:     Encoder, MotorDriver, RcReceiver（+ 外部lib: VoltageSensor, mirs_bmx055）
横断:         SystemContext（層を持たず、タスク間共有専用）
              RobotController（調停者。tasks側から使う）
```

### エンコーダ・オドメトリ仕様

- 計数方式はPCNTハードのX2（2逓倍、1024PPR→2048カウント/回転）。A相両エッジ計数・B相レベル方向判定で、旧ソフトISR（`A==B → +1`）と同一式
- `Encoder` 自体は正逆の意味づけを持たず純粋計数。左右の非対称（ミラー実装）は左エンコーダのピン入替で吸収する（`DiffDrive` のreverse指定は未使用。配置確定後に移管）
- 運動学（ROS側と同一式）：`d=(l+r)/2`、`dtheta=(r-l)/wheel_base`
  - **中点法**：`mid=theta+dtheta/2` で並進積分（円弧誤差低減）
  - `theta` は `[-PI, PI]` に正規化（長時間運転のfloat精度劣化防止）
- 確定パラメータ：`wheel_base = 0.38`（フォールバック値。`/params`受信で上書きされる）。路面・機体が変わったら再較正すること。ROS側 `config.yaml` と同値に保つ

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
| `Encoder` | 直交エンコーダ（PCNTハード計数） | 使用中（`control` タスクで使用） |
| `DiffDrive` | IF層：カウント差分→移動距離（速度計算なし） | 未使用（距離変換は当面control内直計算。配置確定後に移管） |
| `OdometryCalculator` | 計算層：移動距離→自己位置・姿勢 | `control` で実行中。poseは `SharedMotion` 経由で `/odom` 配信 |
| `MotorDriver` | デバイス：MD10C単ch出力 | 使用中（`control` タスクの出力段） |
| `DiffMotors` | IF層：モータペア＋極性吸収 | 使用中（`control` タスクの出力段） |
| `SystemContext` | タスク間共有（キュー・mutex） | 使用中 |
| `PIDController` | 計算層：速度PID（ROS2モード用） | 使用中 |
| `VelocityCalculator` | 計算層：カウント→車輪速度 | 使用中（`DiffDrive` への吸収はしない方針） |
| `RobotController` | MANUAL/ROS2調停・ウォッチドッグ | 使用中（改名は保留） |
| `RcReceiver` | プロポPWM読取（GPIO割込み両エッジ計測、毎周期更新） | 使用中 |
| `VoltageSensor` | バッテリー電圧監視（ADC。外部lib: `extra_packages/VoltageSensor` を `lib_extra_dirs` で参照。git管理外） | 使用中 |
| `mirs_bmx055` | IMU（外部lib: `extra_packages/mirs_bmx055` を `lib_extra_dirs` で参照。git管理外） | 使用中 |
| `SafetyEstop` | E-Stop（外部lib: `extra_packages/SafetyEstop` を `lib_extra_dirs` で参照。git管理外） | 無効（回路実装待ち） |

配置方針：`src/`＝アプリの配線（`main.cpp`・tasks）、`lib/`＝プロジェクト内コンポーネント置き場（`BMX055`・`SafetyEstop`・`VoltageSensor` は分離し `extra_packages/` に移管）。層の所属は[層対応表](#層対応表)で管理する。

命名方針：汎用部品（`Encoder`・`MotorDriver`・`PIDController` 等）は共通orgのrepo置きとし、チーム名を付けない。`mirs2605` 冠はIMU・EKF実装・独自msgs等の2605固有に限定する。

### ライブラリ公開手順

`.gitignore` の `!` 否定で「見るものだけ残す」allowlistを使う。submodule/subtree不要。
`tools/allow/` に lib 内製分（3件分離後は10件）を用意。`mirs_bmx055`・`SafetyEstop`・`VoltageSensor` は `extra_packages/` 配下の外部lib（git管理外、`lib_extra_dirs` で参照）のため対象外。

```bash
# 対象lib以外のuntrackedを隠す（例：Encoderのみ見える）
git -c core.excludesFile=tools/allow/Encoder.gitignore status
```

注意：`!` はuntrackedにのみ作用する。trackedファイルの変更分は通常どおり
表示・ステージ対象になるため、変更分の抽出には下記を併用する。

```bash
git add lib/Encoder/
```

### フェイルセーフ

| 条件 | 動作 |
|---|---|
| ROS2指令途絶 | 1秒（`WATCHDOG_TIMEOUT_MS`）で速度指令ゼロ |
| RC信号喪失 | MANUAL時は停止。ROS2モード時はSW検出不能のため奪取不可、watchdogに委ねる |
| agent未接続・切断 | `ros` タスクのみ再接続待ち。`control` は継続 |
| 非常停止・電圧カットオフ | 回路・配線実装前のため無効 |

### スレッドセーフ方針

- タスク間：mailbox（速度指令・深さ1・最新値上書き）とmutex付き共有（パラメータ・テレメトリ）。1構造体に2 writerを置かない
- ハード計数：PCNTはドライバ側でスレッドセーフな取得（`getCount` 等にcritical不要）。RCパルス・エンコーダ(ホスト)のソフト計数はnoInterrupts継続
- 上記により旧来の速度指令・速度変数の競合は構造的に解消した

## 実機投入前に必ず確認・調整すること

- [ ] `hardware_config.hpp` のピン番号を実際の配線に合わせる
- [ ] `wheel_base`（既定0.38・フォールバック値。`/params`受信で上書き）は路面・機体が変わったら再較正する
- [ ] `VOLTAGE_DIVIDER_RATIO` を実際の分圧抵抗値から計算し直す
- [ ] `VOLTAGE_CUTOFF_THRESHOLD` をバッテリー仕様に合わせる（有効化は配線確認後）
- [ ] `RC_PULSE_MIN/MID/MAX` は実測反映済み（890/1496/2100）。プロポ・機体変更時は再確認する
- [ ] E-Stopスイッチの配線極性（`SafetyEstop::begin()` の `FALLING` 設定）を実機に合わせる
- [ ] `platformio.ini` の `platform` バージョンを大会前に固定（pinning）する

## 安全設計メモ

- RC受信ロスト（電波切れ）：MANUAL時は停止。ROS2モード時は継続し、`/cmd_vel` 途絶から1秒（`WATCHDOG_TIMEOUT_MS`）で速度指令がゼロになる
- E-Stop・電圧カットオフは回路・配線実装前のため無効。`TODO.md` で管理する
