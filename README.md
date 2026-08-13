# CuGo V3 + ESP32 micro-ROS ファームウェア

CuGo V3クローラ、MD10Cモータドライバ、MR-8受信機、ESP32(Arduino framework)、
PlatformIO、micro-ROS(ROS 2 Jazzy, Serial transport)を使用した足回り制御ファームウェア。

## 構成

```
platformio.ini          ボード/フレームワーク/micro-ROS設定
include/config.h         ピン割り当て・CuGoパラメータ・制御定数
lib/MotorDriver/          MD10C (PWM+DIR) 制御
lib/Odometry/              エンコーダ読み取り + 差動二輪オドメトリ計算
lib/RcReceiver/             MR-8 (PWM出力) 読み取り
lib/VoltageSensor/           バッテリー電圧監視 (ADC)
lib/SafetyEstop/              E-Stop (割り込み+フェイルセーフ集約)
src/main.cpp                  micro-ROSノード定義、タスク統合
```

## セットアップ手順

```bash
# 依存ライブラリのインストール
pio lib install

# ビルド
pio run

# 書き込み
pio run --target upload

# シリアルモニタ (micro-ROSと同時使用不可、デバッグ時のみ)
pio device monitor
```

## micro-ROS Agent (PC/ROS 2側)

```bash
# Docker利用の場合
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev [ESP32のシリアルポート] -v6

# ネイティブROS 2 Jazzy環境の場合
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

## Topics

| Topic | 型 | 方向 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe |
| `/odom` | `nav_msgs/msg/Odometry` | Publish |
| `/battery_voltage` | `std_msgs/msg/Float32` | Publish |

## 実機投入前に必ず確認・調整すること

- [ ] `config.h` のピン番号を実際の配線に合わせる
- [ ] `CugoParams::REDUCTION_RATIO` を自機のギアボックス仕様に合わせる
- [ ] `CugoParams::WHEEL_RADIUS_*` / `TREAD` を実測して直進性・旋回精度を校正する
      (公式リファレンス値のため、フレーム構成によりズレる可能性あり)
- [ ] `VOLTAGE_DIVIDER_RATIO` を実際の分圧抵抗値から計算し直す
- [ ] `VOLTAGE_WARN_THRESHOLD` / `VOLTAGE_CUTOFF_THRESHOLD` をバッテリー仕様に合わせる
- [ ] `RC_PULSE_MIN/MID/MAX` をMR-8 + MC-8の実測パルス幅で校正する
- [ ] MotorDriver.cpp のLEDC API (Arduino ESP32 core 2.x系 or 3.x系) が
      使用中のcoreバージョンと一致しているか確認する
- [ ] E-Stopスイッチの配線極性 (`SafetyEstop::begin()` の `FALLING` 設定) を実機に合わせる
- [ ] `platformio.ini` の `platform` バージョンを大会前に固定 (pinning) する

## 安全設計メモ

- E-Stopは `g_estop_active` というグローバルフラグで管理され、
  `MotorDriver::setSpeed()` 内で直接参照される。ROS通信やcontrolLoopTaskが
  詰まっても、モータ出力は独立して停止できる設計。
- RC受信ロスト（電波切れ）、E-Stopスイッチ押下、バッテリー電圧低下の
  いずれでも自動的に `g_estop_active = true` となる。
- 起動直後は `MODE_MANUAL` (手動モード) がデフォルト。ROS2モードへの遷移は
  MR-8のモードスイッチCHの状態でのみ行われる。
