# CuGo V3 + ESP32 micro-ROS ファームウェア

CuGo V3クローラ、MD10Cモータドライバ、プロポ受信機、ESP32 (Arduino framework)、
PlatformIO、micro-ROS (ROS 2 Jazzy, Serial transport) を使用した足回り制御ファームウェア。

プロポ単独で走行でき、ROS 2接続時は自律走行とテレメトリ配信を行う。

## 構成

```
platformio.ini          ボード/フレームワーク/micro-ROS設定
include/
  hardware_config.hpp   ピン割り当て・制御定数
src/
  main.cpp              初期化＋タスク生成のみ
  tasks/
    control_task.*      制御（Core0・15ms・ROS非依存）
    ros_task.*          通信・テレメトリ（Core1）
    sensor_task.*       IMU・電圧ポーリング（Core1）
lib/
  Encoder/              直交エンコーダ（A相CHANGE検出）
  DifferentialDrive/    カウント差分→左右の移動距離・速度
  Odometry/             移動距離→自己位置・姿勢
  MotorDriver/          MD10C単ch出力
  DifferentialMotors/   モータペア＋極性吸収
  SystemContext/        タスク間共有（キュー・mutex）
  PIDController/        速度PID（ROS2モード用）
  RobotController/      MANUAL/ROS2調停・ウォッチドッグ
  RcReceiver/           プロポPWM読取
  VoltageSensor/        バッテリー電圧監視（ADC）
  BMX055/               IMU
  SafetyEstop/          E-Stop（回路実装前のため無効）
  MotorController/      旧ペア実装（superseded。main移管完了後に削除予定）
  VelocityCalculator/   旧速度計算（存続。吸収判断は組込時）
test/
  mocks/Arduino.h       ホストテスト用スタブ
  test_odometry/        Encoder・差動・オドメトリの単体テスト
  test_motor/           モータの単体テスト
DESIGN_SYSTEM.md            全体設計（タスク分割・モード・フェイルセーフ）
DESIGN_ENCODER_ODOMETRY.md  エンコーダ・オドメトリ設計
DESIGN_MOTOR.md             モータ3層設計
TODO.md                     不具合・未完成リスト
```

層の考え方（詳細は `DESIGN_SYSTEM.md` の層対応表）：

```
計算層:       Odometry, PIDController
  ↓ 参照のみ（逆流禁止）
IF層:         DifferentialDrive, DifferentialMotors
  ↓ 参照のみ（逆流禁止）
ハード層:     Encoder, MotorDriver, BMX055, RcReceiver, VoltageSensor
```

## セットアップ手順

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

## micro-ROS Agent（PC/ROS 2側）

```bash
# Docker利用の場合
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev [ESP32のシリアルポート] -v6

# ネイティブROS 2 Jazzy環境の場合
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200
```

## Topics

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

## 動作モード

| モード | 速度指令源 | ROS 2 |
|---|---|---|
| MANUAL（プロポ） | スティック → 開ループ直結 | 不要。接続中はテレメトリ継続（マッピング可） |
| ROS2（自律） | `/cmd_vel` → PID閉ループ | 要 |

- 起動直後は `MODE_MANUAL` がデフォルト
- モードSWの立ち上がりエッジでトグル。ROS2走行中もプロポ操作で制御権を奪取できる
- 受信信号が100ms以上途切れた場合は速度指令をゼロにする
- チャンネル割当やピンは `include/hardware_config.hpp` で変更する

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
