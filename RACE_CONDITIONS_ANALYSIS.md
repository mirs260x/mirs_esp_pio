# 競合・論理的不整合の分析結果

## 🔴 重大な問題（Critical）

### 1. **RobotControllerの速度指令への競合アクセス**

**問題箇所**: `robot_ctrl.getLeftVelCmd()` / `getRightVelCmd()`

**競合パス**:
```
Thread 1 (control_loop):           Thread 2 (timer_callback):
robot_ctrl.update()                portENTER_CRITICAL(&controlMux);
  → l_vel_cmd_ = ... (書き込み)     l_vel_cmd = robot_ctrl.getLeftVelCmd() (読み取り)
  → r_vel_cmd_ = ... (書き込み)     r_vel_cmd = robot_ctrl.getRightVelCmd() (読み取り)
                                   portEXIT_CRITICAL(&controlMux);
```

**問題の詳細**:
- `control_loop()`で`robot_ctrl.update()`が`l_vel_cmd_`と`r_vel_cmd_`を更新
- **同時に** `timer_callback()`で`getLeftVelCmd()`と`getRightVelCmd()`が読み取り
- `controlMux`で保護されていない！

**影響**:
- 不整合な速度指令データの読み取り（左だけ新しく右が古いなど）
- パブリッシュされるテレメトリデータが不正確

**修正方法**:
```cpp
// RobotControllerクラスにミューテックスを追加
class RobotController {
private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    
public:
    double getLeftVelCmd() const {
        portENTER_CRITICAL(&mux_);
        double result = l_vel_cmd_;
        portEXIT_CRITICAL(&mux_);
        return result;
    }
    
    void update(...) {
        portENTER_CRITICAL(&mux_);
        // 速度指令更新
        portEXIT_CRITICAL(&mux_);
    }
};
```

---

### 2. **RobotControllerへの複数経路からの書き込み競合**

**問題箇所**: `robot_ctrl.updateRos2Command()` と `robot_ctrl.update()`

**競合パス**:
```
Thread 1 (cmd_vel_callback):       Thread 2 (control_loop):
robot_ctrl.updateRos2Command()     robot_ctrl.update()
  → linear_x_ = ... (書き込み)        → updateRos2WheelCommands()
  → angular_z_ = ... (書き込み)          → r_vel_cmd_ = linear_x_ + ... (読み取り)
  → last_ros2_cmd_time_ = ...            → l_vel_cmd_ = linear_x_ - ... (読み取り)
```

**問題の詳細**:
- `cmd_vel_callback()`（ROS2コールバック）が`linear_x_`と`angular_z_`を書き込み
- **同時に** `control_loop()`の`updateRos2WheelCommands()`が読み取り
- 保護なし

**影響**:
- `linear_x_`が新しく`angular_z_`が古い状態で計算される可能性
- 意図しない速度指令が生成される

---

### 3. **グローバル変数 `l_vel`, `r_vel` への競合書き込み**

**問題箇所**: `l_vel`, `r_vel`

**競合パス**:
```
Thread 1 (control_loop):           Thread 2 (timer_callback):
vel_calc.calculateBothWheels()     portENTER_CRITICAL(&controlMux);
  → l_vel = ... (書き込み)          double current_l_vel = l_vel; (読み取り)
  → r_vel = ... (書き込み)          double current_r_vel = r_vel; (読み取り)
                                   portEXIT_CRITICAL(&controlMux);
```

**問題の詳細**:
- `control_loop()`が`l_vel`, `r_vel`を書き込み（クリティカルセクション外）
- `timer_callback()`が同じ変数を読み取り（クリティカルセクション内だが書き込み側は保護なし）
- **片側だけの保護は無意味**

**影響**:
- テレメトリで報告される速度が不整合
- デバッグ時に誤った速度データを見る

**修正方法**:
```cpp
static void control_loop() {
    // ...
    vel_calc.calculateBothWheels(snap_l, snap_r, prev_count_l, prev_count_r, temp_l_vel, temp_r_vel);
    
    // クリティカルセクションで更新
    portENTER_CRITICAL(&controlMux);
    l_vel = temp_l_vel;
    r_vel = temp_r_vel;
    portEXIT_CRITICAL(&controlMux);
    
    // ...
}
```

---

## 🟡 中程度の問題（Medium）

### 4. **param_callbackでの非アトミック更新**

**問題箇所**: `param_callback()`

**問題の詳細**:
```cpp
void param_callback(const void *msgin) {
    const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;
    
    // これらの更新は非アトミック
    vel_calc.setWheelRadius(p->wheel_radius);      // control_loop()で使用中かも
    robot_ctrl.setWheelBase(p->wheel_base);        // control_loop()で使用中かも
    pid_right.setGains(p->rkp, p->rki, p->rkd);   // control_loop()で使用中かも
    pid_left.setGains(p->lkp, p->lki, p->lkd);    // control_loop()で使用中かも
}
```

**競合パス**:
```
Thread 1 (param_callback):         Thread 2 (control_loop):
vel_calc.setWheelRadius(...)       vel_calc.calculateBothWheels()
  → wheel_radius_ = ... (書き込み)    → ... * wheel_radius_ / ... (読み取り)
```

**影響**:
- パラメータ更新中に制御ループが実行される
- 一時的に不整合なパラメータで計算される
- PIDゲインが途中まで更新された状態で制御が実行される

**修正方法**:
```cpp
void param_callback(const void *msgin) {
    const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;
    
    // 制御ループの実行を一時停止
    portENTER_CRITICAL(&controlMux);
    vel_calc.setWheelRadius(p->wheel_radius);
    robot_ctrl.setWheelBase(p->wheel_base);
    pid_right.setGains(p->rkp, p->rki, p->rkd);
    pid_left.setGains(p->lkp, p->lki, p->lkd);
    portEXIT_CRITICAL(&controlMux);
}
```

---

### 5. **RcReceiverへの並行アクセス**

**問題箇所**: `rc_receiver.getPulseWidth()`

**問題の詳細**:
```
Thread 1 (control_loop):           Thread 2 (timer_callback):
robot_ctrl.update()                rc_debug_msg.data.data[0] = 
  → rc_receiver.getPulseWidth()      rc_receiver.getPulseWidth(CH_LEFT);
```

**影響**:
- `RcReceiver`内部の状態が競合する可能性
- RcReceiverの実装次第で問題が顕在化

**要確認**: RcReceiverクラスがスレッドセーフかどうか

---

## 🟢 軽微な問題（Minor）

### 6. **BMX055への並行アクセス**

**問題箇所**: `timer_callback()`内の`bmx055.update()`

**問題の詳細**:
- `bmx055.update()`はI2C通信を含む
- `control_loop()`も（将来的に）BMX055にアクセスする可能性

**現状**: 現在は`timer_callback()`からのみアクセスなので問題なし

**将来の懸念**: 制御ループでIMUデータを使うようになると競合

---

### 7. **control_loop_flagの不要なクリティカルセクション**

**問題箇所**: `loop()`内のフラグチェック

**現状のコード**:
```cpp
portENTER_CRITICAL(&controlMux);
bool should_run = control_loop_flag;
if (should_run) {
    control_loop_flag = false;
}
portEXIT_CRITICAL(&controlMux);
```

**問題の詳細**:
- `bool`のread-modify-writeはESP32でアトミック
- クリティカルセクションは不要（ただし、安全のため残しても良い）

**改善案**（オプション）:
```cpp
// volatileなboolのアトミック操作
bool should_run = control_loop_flag;
if (should_run) {
    control_loop_flag = false;
    control_loop();
}
```

---

## 📊 問題の優先度まとめ

| 問題 | 深刻度 | 発生確率 | 影響 | 修正優先度 |
|------|--------|----------|------|------------|
| RobotController速度指令への競合 | 🔴 高 | 高 | データ不整合 | ⭐⭐⭐⭐⭐ |
| RobotController書き込み競合 | 🔴 高 | 中 | 意図しない動作 | ⭐⭐⭐⭐⭐ |
| l_vel/r_vel競合書き込み | 🔴 高 | 高 | テレメトリ不正確 | ⭐⭐⭐⭐ |
| param_callback非アトミック更新 | 🟡 中 | 低 | 一時的な不整合 | ⭐⭐⭐ |
| RcReceiver並行アクセス | 🟡 中 | 中 | 不明 | ⭐⭐ |
| BMX055並行アクセス | 🟢 低 | 低 | なし（現状） | ⭐ |
| control_loop_flag | 🟢 低 | なし | なし | - |

---

## 🛠️ 推奨修正アプローチ

### アプローチ1: クリティカルセクションの追加（簡単）

各競合箇所に`portENTER_CRITICAL` / `portEXIT_CRITICAL`を追加。

**メリット**: 実装が簡単、確実
**デメリット**: クリティカルセクションが多くなる、パフォーマンス影響

### アプローチ2: データのローカルコピー（推奨）

```cpp
static void control_loop() {
    // ローカル変数で計算
    double temp_l_vel, temp_r_vel;
    vel_calc.calculateBothWheels(..., temp_l_vel, temp_r_vel);
    
    // 一度にアトミック更新
    portENTER_CRITICAL(&controlMux);
    l_vel = temp_l_vel;
    r_vel = temp_r_vel;
    portEXIT_CRITICAL(&controlMux);
}
```

**メリット**: クリティカルセクションが短い、パフォーマンス良好
**デメリット**: コードが少し長くなる

### アプローチ3: RobotControllerのスレッドセーフ化（最適）

RobotControllerクラス内部で排他制御を実装。

**メリット**: カプセル化、再利用性向上
**デメリット**: ライブラリの変更が必要

---

## 🎯 次のステップ

1. **重大な問題3つを修正**（優先度⭐⭐⭐⭐⭐/⭐⭐⭐⭐）
2. RcReceiverのスレッドセーフ性を確認
3. 修正後の動作確認とテスト
4. ドキュメント更新
