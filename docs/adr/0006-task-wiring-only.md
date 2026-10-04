# 0006: タスクの配線専任化（推定ファサード＋発行プラグイン＋#if廃止）

- Status: 採用
- Date: 2026-10-04

## 背景

- `control_task` に状態保持（`LoopState`）・EKF融合・速度換算が入り、`ros_task` に発行関数群と `#if ENABLE_IMU` が散在していた。機能の増減にコード除去と分岐整理が伴い、読みづらい。
- 一方でセンサ側の `Registry` は登録1行で増減できており、好評。

## 決定

- 状態推定の判断を `lib/PoseEstimator`（計算層ファサード）に集約する。`control_task` は読取→投入→取出しのみ。Arduino/FreeRTOS非依存にし、host試験する。
- 発行側にも `src/publishers/IPublisherPlugin` を設け、`ros_task` は登録リスト＋送受信ループのみにする（micro-ROS依存のため `src/` 配置）。
- 機能の有無はプリプロセッサではなく「登録の有無＋実行時判定」で表現する。不在時無発行を各プラグインの責務にし、`#if` は `.cpp` から除去する。`ENABLE_EKF` のみ動作切替として残す（`PoseEstimator` への注入値）。
- 例外（C++例外はFW無効）の代わりに、ガード節＋境界での集約処理とする。libは判定のみ返し、ログ・無効化・継続判断はtasks側に寄せる。

## 結果

- IMU除去＝`ImuPlugin` 登録1行＋ `extra_packages/imu` 除外だけ。トピック追加＝発行登録1行。
- 代償：`EstimatorImu` 等の境界値型が増える。変換はタスク側の配線として明示する。
