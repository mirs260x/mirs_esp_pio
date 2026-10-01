#pragma once

// 制御タスク（Core0・15ms・高優先度・ROS非依存）。
// 内部は5ステージのパイプライン：
//   指令受付 → パラメータ適用 → 調停 → 状態推定 → 制御・出力 → テレメトリ
// 計算・調停・出力・共有の実体は各層（RobotController・計算層・DiffMotors・
// SystemContext）が持ち、本ファイルは配線と実行順序のみ担当する。
// ROSの生死に関わらず単独で完結する。
void controlTask(void *arg);

// オドメトリ原点リセット要求（ros側 `/reset_odometry` から呼ぶ）。
// 停止中に呼ぶこと。次 control 周期で x/y/theta をゼロ化する
//（カウント参照は継続するため速度・EKF twist に段差は出ない）。
void requestOdometryReset();
