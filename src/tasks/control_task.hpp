#pragma once

// 制御タスク（Core0・15ms・高優先度・ROS非依存）。
// RC読取→調停（RobotController）→PID/直結→モータ出力。
// ROSの生死に関わらず単独で完結する。旧main.cppの制御経路を移管したもの。
void controlTask(void *arg);
