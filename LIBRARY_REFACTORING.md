# ライブラリ化リファクタリング完了

## 概要

main.cppの機能を4つの独立したライブラリに分離しました。これにより、コードの再利用性、保守性、テスト容易性が大幅に向上しました。

## 作成したライブラリ

### 1. VelocityCalculator (`lib/VelocityCalculator/`)
**機能**: エンコーダーカウントから車輪速度を計算

**主要メソッド**:
- `VelocityCalculator(counts_per_rev, wheel_radius, dt_sec)` - コンストラクタ
- `setWheelRadius(radius)` - 車輪半径を動的更新
- `setDeltaTime(dt_sec)` - サンプリング周期を動的更新
- `calculate(current_count, prev_count)` - 単輪の速度計算
- `calculateBothWheels(...)` - 左右両輪の速度を一度に計算

**使用例**:
```cpp
VelocityCalculator vel_calc(4096.0, 0.04, 0.015);
vel_calc.calculateBothWheels(snap_l, snap_r, prev_l, prev_r, vel_l, vel_r);
```

### 2. PIDController (`lib/PIDController/`)
**機能**: 汎用PID制御

**主要メソッド**:
- `PIDController(kp, ki, kd)` - コンストラクタ
- `setGains(kp, ki, kd)` - PIDゲインを動的更新
- `setOutputLimits(min, max)` - 出力範囲を設定
- `compute(setpoint, measured)` - PID制御計算実行
- `reset()` - 積分項と微分項をリセット

**使用例**:
```cpp
PIDController pid_right(80.0, 30.0, 8.0);
double pwm = pid_right.compute(vel_cmd, vel_actual);
if (vel_cmd == 0.0) pid_right.reset();
```

### 3. RobotController (`lib/RobotController/`)
**機能**: 上位制御（RC/ROS2モード切替、ウォッチドッグ、速度指令統合管理）

**主要メソッド**:
- `RobotController(rc_receiver, wheel_base, max_linear_speed, watchdog_timeout)` - コンストラクタ
- `updateRos2Command(linear_x, angular_z)` - ROS2から速度指令を更新
- `update(ch_left, ch_mode_sw, ch_right, rc_signal_timeout)` - RC入力とモード切替を処理
- `getLeftVelCmd()` / `getRightVelCmd()` - 速度指令を取得
- `getControlMode()` - 現在の制御モード取得（MANUAL / ROS2）
- `setWheelBase(wheel_base)` - 車輪間距離を動的更新

**使用例**:
```cpp
RobotController robot_ctrl(rc_receiver, 0.38, 0.8f, 1000);
robot_ctrl.update(CH_LEFT, CH_MODE_SW, CH_RIGHT, RC_SIGNAL_TIMEOUT_MS);
double l_vel_cmd = robot_ctrl.getLeftVelCmd();
double r_vel_cmd = robot_ctrl.getRightVelCmd();
```

### 4. MotorController (`lib/MotorController/`) ⭐新規追加
**機能**: モーター制御の抽象化

**主要メソッド**:
- `MotorController(pin_dir_l, pin_pwm_l, pin_dir_r, pin_pwm_r)` - コンストラクタ
- `begin(pwm_freq, pwm_resolution)` - 初期化
- `setLeftMotor(pwm)` - 左モーター制御
- `setRightMotor(pwm)` - 右モーター制御
- `setBothMotors(pwm_l, pwm_r)` - 左右モーター同時制御
- `stop()` - 全モーター停止

**使用例**:
```cpp
MotorController motor_ctrl(PIN_DIR_L, PIN_PWM_L, PIN_DIR_R, PIN_PWM_R);
motor_ctrl.begin(20000, 8);  // 20kHz, 8bit
motor_ctrl.setBothMotors(left_pwm, right_pwm);
```

## micro-ROS非依存の制御ループ

### 重要な変更点

**問題**: 以前の実装では、micro-ROSのタイマーコールバック内で制御ループを実行していたため、micro-ROSが落ちるとロボット全体が停止していました。

**解決策**: 軽量フラグ方式による独立制御ループ

### アーキテクチャ

```cpp
// ハードウェアタイマー割り込み（15ms周期、IRAM実行）
void IRAM_ATTR onControlTimer() {
    // 最小限の処理：フラグを立てるだけ
    portENTER_CRITICAL_ISR(&controlMux);
    control_loop_flag = true;
    portEXIT_CRITICAL_ISR(&controlMux);
}

// メインループ（通常コンテキスト）
void loop() {
    // micro-ROS通信処理（ノンブロッキング、1ms）
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    
    // 制御フラグをチェック
    if (control_loop_flag) {
        control_loop_flag = false;
        control_loop();  // RC入力、速度計算、PID制御、モーター出力
    }
}
```

### 設計の利点

1. **割り込みハンドラの最小化**
   - 割り込み内では**フラグ設定のみ**（数マイクロ秒）
   - 実処理は通常コンテキストで実行
   - 他の割り込みや通信をブロックしない

2. **micro-ROSとの協調**
   - `loop()`内で`rclc_executor_spin_some()`を短い間隔（1ms）で呼び出し
   - micro-ROS通信と制御ループが交互に実行される
   - 通信処理を妨げない

3. **フェイルセーフ動作**
   - micro-ROSが落ちても`loop()`は動作し続ける
   - タイマー割り込みでフラグが立ち、制御ループが実行される
   - RCコントローラーから操作可能

### タイミング図

```
時刻      0ms    1ms    15ms   16ms   30ms   31ms
         |      |      |      |      |      |
Timer:                 ↑            ↑
         (割り込み)    flag=1       flag=1

Loop:    ROS→ctrl→ROS→ctrl→ROS→ctrl
         |1ms|    |1ms|    |1ms|
         
ctrl: 制御ループ実行（RC、速度計算、PID、モーター出力）
ROS:  micro-ROS通信処理（rclc_executor_spin_some）
```

### なぜこの方式か

**❌ 避けた方式：割り込み内で全処理**
```cpp
void IRAM_ATTR onControlTimer() {
    robot_ctrl.update(...);      // RC入力読取（数百μs）
    vel_calc.calculate(...);      // 速度計算（数十μs）
    pid_right.compute(...);       // PID計算（数十μs）
    ledcWrite(...);               // PWM出力
}
```
→ 割り込みが長時間（1ms以上）実行されると：
- 他の割り込み（エンコーダー、RC受信）が遅延
- micro-ROS通信がブロックされる
- シリアル通信がドロップする可能性

**✅ 採用した方式：軽量フラグ + loop()実行**
```cpp
void IRAM_ATTR onControlTimer() {
    control_loop_flag = true;  // 数μsで完了
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    if (control_loop_flag) {
        control_loop();  // 通常コンテキストで実行
    }
}
```
→ メリット：
- 割り込みハンドラは即座に完了（<10μs）
- micro-ROS通信と制御が協調動作
- 他の割り込みを妨げない

## アーキテクチャ図

```
┌─────────────────────────────────────────────────────┐
│          Hardware Timer (15ms, 独立動作)             │
│  ┌────────────────────────────────────────────────┐ │
│  │ 1. RC入力読取 (RobotController)                │ │
│  │ 2. モード切替 (MANUAL ⇔ ROS2)                  │ │
│  │ 3. 速度計算 (VelocityCalculator)               │ │
│  │ 4. PID制御 (PIDController × 2)                 │ │
│  │ 5. モーター出力                                 │ │
│  └────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│    micro-ROS Timer (オプション、通信依存)            │
│  ┌────────────────────────────────────────────────┐ │
│  │ 1. センサーデータ読取 (BMX055)                 │ │
│  │ 2. テレメトリパブリッシュ                      │ │
│  │    - /encoder, /vel, /vlt, /rc_debug          │ │
│  │    - /imu/data_raw, /imu/mag                  │ │
│  └────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
                         ↓
                   ROS2 Network
```

## コンパイル結果

```
✅ コンパイル成功
RAM:   18.2% (59,476 / 327,680 bytes)
Flash: 32.3% (423,518 / 1,310,720 bytes)

作成ライブラリ数: 4つ
- VelocityCalculator
- PIDController
- RobotController
- MotorController (新規)
```

## リファクタリングで改善された点

### 1. **コードの分離と明確化**
- モーター制御が`MotorController`クラスに抽象化
- ピン番号や方向設定の詳細が隠蔽され、main.cppがよりシンプルに

### 2. **定数の明確化**
- `const`修飾子の適切な使用
- マジックナンバーの削減

### 3. **コメントの改善**
- 各処理ブロックの目的を明確化
- 初期化処理の順序と理由を記載

### 4. **エラーハンドリングの準備**
- 各ライブラリが独立しているため、個別のエラーチェックが容易
- 将来的な拡張に対応しやすい構造

## 動作モード

### 1. 手動モード (MANUAL)
- RCコントローラーの左右スティックで左右輪を個別制御
- micro-ROSの状態に関係なく常に動作可能

### 2. ROS2モード (ROS2)
- `/cmd_vel` トピック（geometry_msgs/Twist）から速度指令を受信
- ウォッチドッグ機能: 1秒間通信が途絶えると自動停止
- micro-ROSが落ちた場合、RCコントローラーで手動モードに切替可能

### モード切替
- RCコントローラーのモードスイッチの立ち上がりエッジ（0→1）で切替
- LED等で現在モードを確認可能（`/rc_debug` トピックの6番目の要素: 0=MANUAL, 1=ROS2）

## パラメータ動的更新

`/params` トピック（mirs_msgs/BasicParam）でパラメータを動的更新可能:
- `wheel_radius` → VelocityCalculator
- `wheel_base` → RobotController
- `rkp, rki, rkd` → 右輪PIDゲイン
- `lkp, lki, lkd` → 左輪PIDゲイン

## 今後の拡張性

各ライブラリは独立しているため、以下のような拡張が容易:
- 他のロボットプロジェクトへの移植
- PIDControllerの代わりに別の制御手法を実装
- 異なるエンコーダー仕様への対応（VelocityCalculatorのパラメータ変更のみ）
- オドメトリ計算ライブラリの追加
