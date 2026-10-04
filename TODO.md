# TODO（不具合探索・未完成リスト）

`[P0]` 移管・マージ時の必須、`[P1]` 想定外ケース・堅牢性、`[P2]` 衛生・後回し可。
🔧は実機・実環境が必要、💻は机上（host/コンテナ検証）で完結する。
最終棚卸し：2026-09-15（実機走行確認分を反映。残りは配線・ROS側のみ）。
全体検査・修正反映：2026-10-04（host66件・実機ビルド0/1両方・pytest18件）。
実機/机上分割：2026-10-04。

## P0：移管・マージ時の必須

- [x] 距離変換のIF層への寄せ → 対応済み（`DiffDrive::sample()` に差分一本化。wrap＋反転＋距離をIF層で完結し、`LoopState` は戻り差分を積算するのみ。二重積算の乖離を構造的に解消。`test_odometry` に2件追加）
- [ ] 🔧 電圧カットオフの結線未実施（`/vlt` 監視のみで遮断なし。配線待ち）

### 参考：`VelocityCalculator` と `DiffDrive` の違い（確定）

- `VelocityCalculator`：計算層。純粋な計算関数オブジェクト。生カウント＋前回値を渡すと車輪速度[m/s]を返すだけ。前回値・パラメータの保持は呼び出し側。現行はPIDフィードバックと `/vel` 用に `control` タスクで使用中
- `DiffDrive`：IF層。機構オブジェクト。`Encoder` 2つを保持し前回値を内部管理。差分→移動距離[m]変換とreverse吸収を行う。速度計算は持たない。`update()` するだけ
- 分離維持：速度式は同一だが、計算とIFの分離のため両方残す。`DiffDrive` への吸収は行わない

## P1：想定外ケース・堅牢性

- [x] IMU `frame_id: imu_link` のTF接続 → 対応済み（`mirs_hardware.launch.py` に `base_link`→`imu_link` static TF追加。xyz実測後に更新すること）
- [ ] 🔧 RCの実機確認（RC中立1496・SW 1995読値、mode閾値0.2の動作）
- [ ] 🔧 ros切断→再接続の実機試験（`ros_teardown` 後の再setup、ハンドル枯渇・リークの有無）
- [ ] 🔧 GPIO自前計数での実機確認（4096/rev・正逆・ノイズ誤計数。PCNT撤去後の再確認。`odom_linear_test`・`odom_rotate_test` で実施）

## EKF・IMU活用（2026-10-04追加。Nav2接続の前提）

方針：ESP32＝前段フィルタ（`Mirs2605Ekf`）＋ROS側 `robot_localization` 維持の二段構成。
`ENABLE_EKF 0` が既定（従来動作のまま）。`1` で `/odom` をフィルタ結果で上書きする。

### ファーム側

- [ ] 🔧 BMX055軸合わせ・符号確認（x前・y左・z上（REP-103）の前提。不一致なら `PoseEstimator` 投入前に補正。EKF有効化の必須条件）
- [ ] 🔧 地磁気キャリブ（declination・hard-iron。屋内磁気外乱時は無効化して評価すること。残差1.0rad超は棄却済み）
- [ ] 🔧 Q/R同調（既定は室内低速向け。実機データで調整。`setProcessNoise`・`setMeasurementNoise`）
- [ ] 🔧 `ENABLE_EKF 1` の実機評価（直進・旋回で `/odom` が発散しないこと。`odom_linear_test`・`odom_rotate_test` で確認）

### ROS側の宿題（正本：`../../ws/TODO.md` §4.4。ここには依存関係のみ記録）

- [x] `imu_link` のTF追加 → 対応済み（ws側でstatic TF追加）
- [x] Nav2の `odom_topic` 見直し → 結論：生 `/odom` 維持（ws側に記録）
- [ ] 🔧 ESP32 EKF有効時のROS側EKF再調整（`ENABLE_EKF=1` の実機評価時に実施）
- [x] `/imu/mag` の扱い確定 → 方針確定（ESP32内EKFのみ使用）

## リファクタリング候補（2026-10-04全体検査。`mirs_esp_pio`＋ROS境界）

### P1：堅牢性・安全側への倒し方

- [x] EKFバイアス更新の無条件実行 → 対応済み（`updateYawRateGated()` 追加。|v|0.05超では更新しない。`fuseEkf` はゲート版を使用。`test_ekf` に2件追加）
- [x] エンコーダ二重積算の反転順序不整合 → 対応済み（P0距離変換と同一対応。`sample()` の戻り差分に一本化し、呼び出し側の掛け直しを廃止）

### P2：冗長・死コード・ドリフト

- [x] PCNTドライバ排除 → 対応済み（外部ライブラリ不使用方針。GPIO割込みX4自前計数に統一しhost同一実装に。`docs/adr/0005`。`FW_VERSION` 0.5.0）
- [x] `DiffDrive` の3重API → 対応済み（`update()`・`snapshot()`・`reset()` をhost試験専用としてヘッダに明記。本番は `sample()` のみ）
- [x] `OdometryCalculator::v_linear/v_angular` の死フィールド → 対応済み（`/odom` twistの単一出所に昇格。`publishTelemetry` は再計算値を発行しない）
- [x] パラメータ適用先の拡散 → 対応済み（`applyParamsIfChanged` に適用先一覧を集約コメント。新規consumer追加時の登録先を明記）
- [x] 本番未使用の公開API群 → 対応済み（`stop`・`reset`・`getLinearX/getAngularZ`・`setControlMode` に用途注記。E-Stopなし方針と整合）
- [x] `Registry` の境界チェック欠如 → 対応済み（範囲外は空文字/false返却。`test_registry` に1件追加）
- [x] `RcReceiver` の単一実体制約とchガード → 対応済み（未登録chは0/false返却に修正。単一実体はヘッダ注記のまま。`test_robot` に1件追加）
- [x] 16bit折返し前提の陳腐化 → 対応済み（2026-10-04にPCNT自体を撤去。`wrapDelta` はint32ラップ吸収＋最終防御として残す）
- [x] ホストモックの逓倍乖離 → 対応済み（2026-10-04にGPIO自前計数へ統一しhostと同一実装に。X4の正しさもhostで検証可。`test_glitch_edges_ignored` 追加）
- [x] `/imu` orientation未設定 → 対応済み（単位四元子で初期化。`covariance[0]=-1` の未知扱いは維持）
- [x] EKF有効時の `odom.update` 空転 → 対応済み（`ENABLE_EKF=1` 時は更新スキップ。0/1両ビルド成功）
- [x] `FW_VERSION` 未更新 → 対応済み（0.5.0に更新）
- [x] `main.cpp` トピックコメントの欠落 → 対応済み（全トピック記載。`ENABLE_EKF` 時の `/odom` 注記付き）

## タスク瘦身・枠組み化（2026-10-04実施。`docs/adr/0006`）

方針：機能の有無は登録の有無＋実行時判定で表現し、`#if` は `.cpp` から除去。
`control_task`・`ros_task` は配線専任（投入・取出し・登録リストのみ）。

- [x] `lib/PoseEstimator` 新設（odom・速度・EKFの判断を集約。Arduino非依存でhost試験。`test_estimator` 6件追加）
- [x] `src/publishers` 新設（`Imu`・`Odom`・`Telemetry`。トピック増減は登録1行。`ros_task.cpp` から約150行を移管）
- [x] `#if ENABLE_IMU` 除去（不在時は無効化＋無発行。`ENABLE_IMU` マクロ自体を廃止。`ENABLE_EKF` のみ注入値として残す）
- [x] `velCmdToDuty` を `RobotController::toDuty` に移管（max<=0時は安全側0。`test_robot` に2件追加）
- [x] 不要コード削除（`getLinearX/getAngularZ`・`DiffDrive::reset`・`BMX055` 未定義宣言2件。参照ゼロを確認）
- [x] `warn_unused_result` 警告の除去（`ignoreResult` 集約。実機ビルド警告ゼロ）
- [ ] 💻 電圧異常の不可視化（`VoltagePlugin.cpp:15-27`。`begin` 常時true・`readVoltage` にエラー戻りなし。ADC断線時は0V沈黙配信。外部repo改修が必要）
- [ ] 💻 PIDのD項キック（`PIDController.cpp:48-89`。誤差微分のため `/cmd_vel` ステップでDスパイク。出力clamp済みのため実害小。必要なら測定値微分・setpointフィルタ化）

## 未確定事項（旧DESIGN_SYSTEM.md §8より移管）

- [ ] 🔧 タスク優先度・コア割当の確定（現行：control 10/Core0、ros 5/Core1、sensor 3/Core1。実機評価待ち）
- [ ] 💻 mutex範囲の確定（現行：mutex 5msタイムアウト。保持処理は単純コピーのためコード監査で確定可。キューはmailbox化によりdepth確定済み）
- [ ] 🔧 電圧カットオフの有効化時期（配線待ち。非常停止は廃止済みのため対象外）
