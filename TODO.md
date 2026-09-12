# TODO（不具合探索・未完成リスト）

現構成のまま洗い出したもの。実装は完成していない前提。
`[P0]` 移管・マージ時の必須、`[P1]` 想定外ケース・堅牢性、`[P2]` 衛生・後回し可。

## P0：移管・マージ時の必須

- [x] `main.cpp` → `src/tasks/` 移管（旧経路のまま振る舞い同一。新クラス群への切替は別途）
- [x] `ESP32Encoder` の `lib_deps` 残骸削除
- [ ] 新クラス群への切替（`Encoder`・`DifferentialDrive`・`Odometry`＋`DifferentialMotors`、旧 `MotorController` 削除、2048修正に伴うPID・`/vel` 再確認）
- [ ] `ros_setup()` の戻り値全無視（`src/tasks/ros_task.cpp`）。agent不在起動時の動作は未定義のまま
- [ ] `SystemContext::set/get` のmutex nullガードなし。`begin()` 前呼び出しでクラッシュする
- [ ] 非常停止・電圧カットオフの結線未実施（`SafetyEstop` 無効、`ENABLE_VOLTAGE_CUTOFF 0`）
- [ ] `VelocityCalculator` の吸収判断（`DifferentialDrive` の速度出力で代替可否）

## P1：想定外ケース・堅牢性

- [ ] int32カウンタ差分の符号オーバーフローUB（`cur - last`）。約6500カウント/秒で約3.8日連続運転が目安。int64蓄積 or ラップアラウンド対策
- [ ] `/params` 無検証適用（`main.cpp:172-182`）。NaN・負値・異常ゲインで不安定化しうる。範囲チェックを入れる
- [ ] RC必須によるROS2停止（`RobotController::update:38-50`）。TX OFFではROS2モードも止まる。緩和可否は下記未確定事項と統合して判断
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
- [ ] READMEの仕様章と実装のドリフト管理（旧DESIGN_*.mdは本READMEに統合済み）

## 未確定事項（旧DESIGN_SYSTEM.md §8より移管）

- [ ] タスク優先度・コア割当の確定（現行：control 10/Core0、ros 5/Core1、sensor 3/Core1）
- [ ] キュー深度・mutex範囲の確定（現行：キューdepth 4、mutex 5msタイムアウト）
- [ ] ROS不在時の初回起動シーケンス（agent待ちの上限・リトライ間隔）
- [ ] RC信号喪失時のROS2モード継続可否（現行はRC必須＝停止する。緩和するか）
- [ ] 電圧カットオフ・非常停止の有効化時期
