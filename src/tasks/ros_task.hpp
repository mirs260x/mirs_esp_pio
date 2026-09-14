#pragma once

// 通信タスク（Core1・中優先度）。
// micro-ROS agent接続、/cmd_vel受信→キュー投入、テレメトリ発行、/params反映。
// agent未接続・切断時は再接続待ちのみ行い、他タスクに影響させない。
void rosTask(void *arg);
