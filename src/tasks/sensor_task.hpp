#pragma once

// センサタスク（Core1・15ms・低優先度）。
// IMU・電圧のポーリング専用。I2Cはタイムアウト付きで呼び、
// 制御周期を乱さない。結果はSystemContextへ書込む。
void sensorTask(void *arg);
