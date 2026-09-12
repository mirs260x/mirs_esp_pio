# TODO（不具合探索・未完成リスト）

現構成のまま洗い出したもの。実装は完成していない前提。
`[P0]` 移管・マージ時の必須、`[P1]` 想定外ケース・堅牢性、`[P2]` 衛生・後回し可。

## P0：移管・マージ時の必須

- [ ] `main.cpp` → `src/tasks/` 移管未実施（`control_loop`・`ros_setup`・`timer_callback`・`alloc_messages`）。現FWは旧経路で動作中のため、新旧二重管理状態
- [ ] `ros_setup()` の戻り値全無視（`src/main.cpp:289-316`）。agent不在起動時の動作は未定義。`delay(2000)` ブロッキングも `ros_task` 隔離時に除去すること
- [ ] `SystemContext::set/get` のmutex nullガードなし（`lib/SystemContext/SystemContext.cpp`）。`begin()` 前呼び出しでクラッシュする
- [ ] 非常停止・電圧カットオフの結線未実施（`SafetyEstop` 無効、`ENABLE_VOLTAGE_CUTOFF 0`）。`g_estop_active` の読み手がゼロ（旧 `MotorDriver` 削除済みのため）
- [ ] 旧 `MotorController` の削除（`main.cpp` 移管時。同時に `MotorDriver`＋`DifferentialMotors` へ切替）
- [ ] `VelocityCalculator` の吸収判断（`DifferentialDrive` の速度出力で代替可否）
- [ ] `ESP32Encoder` の `lib_deps` 残骸確認・削除（`platformio.ini:23-24`。参照元ゼロ）

## P1：想定外ケース・堅牢性

- [ ] int32カウンタ差分の符号オーバーフローUB（`cur - last`）。約6500カウント/秒で約3.8日連続運転が目安。int64蓄積 or ラップアラウンド対策
- [ ] `/params` 無検証適用（`main.cpp:172-182`）。NaN・負値・異常ゲインで不安定化しうる。範囲チェックを入れる
- [ ] RC必須によるROS2停止（`RobotController::update:38-50`）。TX OFFではROS2モードも止まる。緩和可否は `DESIGN_SYSTEM.md` 未確定4と統合して判断
- [ ] `setReversed` 途中変更の基準ずれ（`DifferentialDrive`・`DifferentialMotors`）。走行前設定の徹底 or 変更時リセットのガード
- [ ] `SystemContext` キュー溢れポリシー（depth 4、溢れ時黙って破棄）。溢れ検出・通知の要否
- [ ] `getParams`・`getTelemetry` のmutexタイムアウト時ゼロ値返却（黙って既定値）。呼び側が気づけない
- [ ] `DifferentialMotors` のdouble→int切捨て（微小duty消失）。四捨五入 or デッドバンド明示
- [ ] `RcReceiver` の有効範囲二重定義（ISR側800〜2200 vs `pulseToNormalized`・config 1000〜2000）。一元化
- [ ] `VoltageSensor::readVoltage` のブロッキング読取（16サンプル平均＋EMA）。`sensor_task` 移管で解消予定
- [ ] PIDのアンチワインドアップ・dtなし（`lib/PIDController/`）。周期変更で特性変化
- [ ] `bmx055.begin()` の戻り値無視（`src/main.cpp:386`）。IMU不在でも黙って継続する
- [ ] `alloc_messages()` のmalloc nullチェックなし（`src/main.cpp:322-361`）
- [ ] IMU `frame_id: imu_link` のURDF不在（ROS側TF不整合）

## P2：衛生・後回し可

- [ ] `test/mocks/Arduino.h` の共有スタブ：odometry側で `digitalWrite` 等未定義のまま。将来参照したらリンクエラーになる
- [ ] `DifferentialDrive` が左の `countsPerRev()` のみ参照（左右同一の暗黙前提）
- [ ] 設計書（`DESIGN_*.md` §9手順等）と実装のドリフト管理
