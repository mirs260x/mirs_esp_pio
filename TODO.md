# TODO（不具合探索・未完成リスト）

`[P0]` 移管・マージ時の必須、`[P1]` 想定外ケース・堅牢性、`[P2]` 衛生・後回し可。
最終棚卸し：2026-09-13（実機不要分を一括対応。残りは実機・配線・ROS側のみ）。

## P0：移管・マージ時の必須

- [x] `main.cpp` → `src/tasks/` 移管（旧経路のまま振る舞い同一。新クラス群への切替は別途）
- [x] `ESP32Encoder` の `lib_deps` 残骸削除
- [x] `Encoder` のcontrol組込・PCNT化（`control_task` は `enc_l/enc_r.getCount()` 使用中。左ミラーはピン入替で吸収。`COUNTS_PER_REV=2048` 反映済み）
- [x] 計算層のcontrol実行組込（`VelocityCalculator`・`PIDController` に加え `OdometryCalculator` を `control_loop` で実行。poseは `SharedMotion.odom_*` に格納）
- [x] オドメトリのpublish配線（`/odom`＝`nav_msgs/Odometry`、約67Hz、frame `odom`→`base_link`。TF・URDF側はROS側の宿題として残存）
- [ ] 距離変換のIF層への寄せ（現状control内直計算。`DiffDrive` 配置確定後に移管。方向性未確定のため見送り）
- [ ] 2048修正に伴うPID・`/vel` 再確認（実機。定数は反映済み、走行確認が未済）
- [x] `ros_setup()` の戻り値処理・agent不在動作の定義（`src/tasks/ros_task.cpp` をbool化＋ping待機・切断再接続。待機するのはrosタスクのみ）
- [x] `SystemContext::set/get` のmutex nullガード（`begin()` 前は直読み直書き＋フォールバック既定値維持。タイムアウト時はゼロ値でなく直近値を返却）
- [ ] 非常停止・電圧カットオフの結線未実施（`SafetyEstop` 無効、`ENABLE_VOLTAGE_CUTOFF 0`。配線待ち）
- [x] `VelocityCalculator` の位置づけ確定：計算層として存続し、`DiffDrive`（IF層）への吸収はしない方針

### 参考：`VelocityCalculator` と `DiffDrive` の違い（確定）

- `VelocityCalculator`：計算層。純粋な計算関数オブジェクト。生カウント＋前回値を渡すと車輪速度[m/s]を返すだけ。前回値・パラメータの保持は呼び出し側。現行はPIDフィードバックと `/vel` 用に `control` タスクで使用中
- `DiffDrive`：IF層。機構オブジェクト。`Encoder` 2つを保持し前回値を内部管理。差分→移動距離[m]変換とreverse吸収を行う。速度計算は持たない。`update()` するだけ
- 分離維持：速度式は同一だが、計算とIFの分離のため両方残す。`DiffDrive` への吸収は行わない

## P1：想定外ケース・堅牢性

- [x] int32カウンタの長期オーバーフロー → `wrapDelta`＋int64累積化済み（`VelocityCalculator` にint64版追加、`control` は累積保持。ホストテストでラップ検証）
- [x] `/params` 無検証適用 → 範囲チェック済み（半径(0,0.5]・ベース(0,2.0]・ゲイン有限||≤1000。範囲外は棄却＋ログ）
- [x] RC不要化の実装（ROS2モードはRC信号不要にし、停止判定はwatchdogに一本化。MANUALはRC必須のまま。SW喪失中は切替凍結）
- [x] SW押下でROS2からMANUALへ奪取（トグル維持。ROS2からのトグル先は必ずMANUALのため押下で奪取が保証される）
- [x] `setReversed` 途中変更の基準ずれ → 解決不要（`src/` からの呼出しなし。`begin` 時の固定値のみ使用）
- [x] `SystemContext` キュー溢れポリシー → 深さ1のmailbox化済み（常に最新値上書き。溢れ時の黙殺は構造的に発生しない）
- [x] `getParams`・`getMotion`・`getSensor` のmutexタイムアウト時ゼロ値返却 → 直近値返却に変更済み
- [x] `DiffMotors` のdouble→int切捨て → 四捨五入に変更済み（ホストテスト追加）
- [x] `RcReceiver` の有効範囲二重定義 → `RC_PULSE_MIN/MID/MAX`（890/1496/2100）に一元化済み（ISR側直書き排除）
- [x] `VoltageSensor::readVoltage` のブロッキング読取 → 完了扱い（`sensor_task` 分離で制御周期への影響は解消済み。分散化は見送り）
- [x] PIDのアンチワインドアップ・dtなし → `compute(setpoint, measured, dt_sec)` 化済み（条件付き積分＋dt基準の微分。`control` は `DT_SEC` 渡し）
- [x] `bmx055.begin()` の戻り値未チェック → 警告ログ追加済み（不在でも他センサ継続。`isInitialized()` ガード継続）
- [x] `alloc_messages()` のmalloc nullチェックなし → bool化＋null検査済み
- [ ] IMU `frame_id: imu_link` のURDF不在（ROS側TF不整合。ROS側リポジトリの宿題）
- [ ] PCNT/RC割込みの実機確認（エンコーダ正逆・`/vel` 符号、RC中立1496・SW 1995読値、mode閾値0.2の動作）
- [ ] ros切断→再接続の実機試験（`ros_teardown` 後の再setup、ハンドル枯渇・リークの有無）
- [x] watchdog修正の単体テスト欠如 → `test_robot` 追加済み（toggle・watchdog・RC不要化をホスト検証。`lib_ignore` から除外）

## P2：衛生・後回し可

- [x] `test/mocks/Arduino.h` の共有スタブ → `millis/micros` 宣言・`constrain`・`INPUT_PULLDOWN`・`<cstdlib>` を追加済み（定義は各TU持ちのまま）
- [x] `DiffDrive` が左の `countsPerRev()` のみ参照 → 左右それぞれの自前値を使用に変更済み
- [x] READMEの仕様章と実装のドリフト管理 → 追従済み（PCNT/RC割込み・mailbox・`/odom`・RC実測・スレッドセーフ章。MANUAL「開ループ直結」表記のみ意図未確定で残存）
- [x] GPIO割込み 3ch・PCNT 2unitのハード資源台帳化 → READMEアーキテクチャ章に記載済み（RMT方式は320ms更新で100msタイムアウトに間に合わないため廃止）

## 未確定事項（旧DESIGN_SYSTEM.md §8より移管）

- [ ] タスク優先度・コア割当の確定（現行：control 10/Core0、ros 5/Core1、sensor 3/Core1。実機評価待ち）
- [ ] mutex範囲の確定（現行：mutex 5msタイムアウト。キューはmailbox化によりdepth確定済み）
- [x] ROS不在時の初回起動シーケンス → 定義済み（ping待機→setup→2s毎ping・3回失敗で再接続。rosタスクのみ待機）
- [x] RC信号喪失時のROS2モード継続可否 → 方針確定・実装済み（RC不要化。停止判定はwatchdogに一本化）
- [ ] 電圧カットオフ・非常停止の有効化時期（配線待ち）
