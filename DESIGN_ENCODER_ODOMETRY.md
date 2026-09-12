# エンコーダ・オドメトリ設計書

## 1. 目的

- エンコーダ読み取りを `Encoder` クラスに集約し、差動二輪をそのインスタンス2つで表現する
- ESP側でオドメトリ（`x, y, theta`）を計算できる形に戻す（`lib/Odometry/` は旧設計のため使わない）
- 現行コードの逓倍数と `COUNTS_PER_REV` の不整合を解消する

## 2. 現状の問題整理

| # | 事象 | 場所 |
|---|---|---|
| 1 | ISRはA相CHANGEのみ検出（X2：2逓倍相当）なのに、`COUNTS_PER_REV=4096`（1024×4の前提） | `main.cpp:99-112`、`hardware_config.hpp:16` |
| 2 | エンコーダ処理が `main.cpp` に直書き（グローバル `count_l/count_r`＋ISR関数2つ） | `main.cpp:56,99-112` |
| 3 | 速度計算は `VelocityCalculator` が担当するが、カウント取得はクラスの外 | `main.cpp:139` |
| 4 | 旧 `lib/Odometry/` は `CugoParams` 参照で死んでいる。`main.cpp.bak` と共に旧世代の残骸 | `lib/Odometry/`（削除済：`main.cpp.bak` のみ処分済み） |

X2（2逓倍）の場合、1024PPRなら1回転あたりのカウントは **2048** が正しい。
4096のまま運用すると速度・距離が **1/2倍** に systematic にずれる。

## 3. 設計方針

1. **1エンコーダ＝1オブジェクト**：ピンもカウントも方向判定も `Encoder` が持つ。`main.cpp` は触らない
2. **差動二輪＝`Encoder` ×2**：左右のインスタンスを `Odometry`（新設）に渡す。以後の拡張（4輪・メカナム）はインスタンス追加＋運動学差し替えで対応
3. **検出方式はA相CHANGEのみに固定**：X2（2逓倍、1024PPR→2048カウント/回転）。他方式は持たない
4. **購読系は維持**：`/cmd_vel`・`/params` の subscription 構成は変えない（本設計の対象外）
5. **旧 `lib/Odometry/` は使わない**：新規 `Odometry` クラスを起こし、旧ディレクトリは削除する

## 4. `Encoder` クラス設計

```cpp
class Encoder {
public:
    static constexpr uint32_t PULSES_PER_REV = 1024;
    static constexpr uint32_t MULTIPLIER = 2;  // X2：2逓倍

    explicit Encoder(uint8_t pin_a, uint8_t pin_b, bool reverse = false);
    void begin();               // pinMode + attachInterrupt(A相CHANGE)
    int32_t getCount() const;   // スナップショット取得
    void reset();
    uint32_t countsPerRev() const;  // 2048
private:
    static void IRAM_ATTR isrA(void *arg);
    void handleA();
    uint8_t pin_a_, pin_b_;
    bool reverse_;
    volatile int32_t count_;
};
```

### 4.1 方向判定とreverseフラグ

- 方向判定式は現行と同一（A変化後に `A==B → +1`、不一致 → `-1`）とし、左右の非対称はコンストラクタの `reverse` フラグで吸収する（左 `true`・右 `false` が現行動作と一致。`pio test` で検証済み）
- B相は読むだけでエッジ検出しないため、B線ノイズで誤カウントしない
- `count_` は `volatile int32_t`。取得は `portENTER_CRITICAL` でスナップショット
- ISR内は加減算のみ（現行どおり最小限）
- `RcReceiver` と同じ `attachInterruptArg`＋`void* arg` 方式でインスタンスを特定する

## 5. 差動二輪としての構成

```cpp
Encoder enc_l(PIN_ENC_A_L, PIN_ENC_B_L, true);   // 左はreverse
Encoder enc_r(PIN_ENC_A_R, PIN_ENC_B_R, false);
DifferentialDrive dd(enc_l, enc_r);  // 参照で保持
```

- `main.cpp` のグローバル `count_l/count_r` とISR関数2つは削除し、上記2行＋`begin()` に置換する
- `VelocityCalculator` は当面残す：`calculateBothWheels()` の引数に `enc.getCount()` のスナップショットを渡す形に変えるだけで済む
  - 将来的には `Odometry` が速度も出すため、`VelocityCalculator` は吸収・削除候補

## 6. `Odometry` クラス設計（新設、旧ディレクトリは削除）

```cpp
class Odometry {
public:
    Odometry(Encoder& left, Encoder& right);
    void setWheelParams(double wheel_radius, double wheel_base);
    void setCountsPerRev(double cpr);  // /params 経由の動的更新用（将来）
    void update();                     // dtは内部Timer間隔 or 引数
    void reset();
    float x, y, theta;                 // 積算位置・姿勢
    float v_linear, v_angular;         // 最新速度
private:
    Encoder &left_, &right_;
    int32_t last_l_, last_r_;
    double wheel_radius_, wheel_base_;
};
```

- 運動学はROS側（`odometry_publisher.cpp`）と同一式にし、両者の二重管理を将来解消する足がかりにする
  - `d = (l+r)/2`、`dtheta = (r-l)/wheel_base`
  - **中点法**：`mid = theta + dtheta/2`、`x += d*cos(mid)`、`y += d*sin(mid)`（円弧誤差を低減。更新後theta方式は内回りバイアスが溜まるため不採用）
  - `theta` は `[-PI, PI]` に正規化（長時間運転でのfloat精度劣化防止）
- パラメータはコンストラクタ注入＋setter（`/params` 受信時の `param_callback` から反映できるよう、`vel_calc`・`robot_ctrl` と同じ形にする）
- `CugoParams`・左右別半径・`TREAD` 等の旧名称は持ち込まない。`wheel_radius`・`wheel_base` に統一（ROS側 `config.yaml` と同名）
- 公開形式は当面 **`/encoder` 生カウントのまま**（ROS側オドメトリが動いているため）。ESP側 `Odometry` の値はまず `/vel` 拡張 or デバッグ出力で検証し、一致確認後に `nav_msgs/Odometry` 発行へ切替える二段階移行とする

## 7. データフロー（移行後）

```
[ISR] enc_l/enc_r が count_ 更新
  ↓ (15ms制御ループ)
control_loop():
  snap_l = enc_l.getCount() / snap_r = enc_r.getCount()  (critical内)
  vel_calc.calculateBothWheels(...)   // 当面維持
  odom.update()                        // 新規：積算のみ、出力はまだ使わない
  PID → MotorController                // 変更なし
[ROS timer]
  /encoder に snap 値を発行            // 変更なし
  /params 購読 → vel_calc/robot_ctrl/odom に反映
  /cmd_vel 購読 → robot_ctrl           // 変更なし
```

## 8. 購読（subscription）の扱い

- `/cmd_vel`・`/params` の購読構成・executor登録数は変えない
- `param_callback` の反映先に `odom.setWheelParams()` を追加する（1行）
- 将来 `setCountsPerRev()` を生やす場合は `BasicParam` に項目追加が必要＝`mirs_msgs` 変更になるため、本設計では見送り（`count_per_rev` はFW側 `countsPerRev()` の既定値を使用）

## 8.5 確定パラメータ（外部基準で較正済み）

- `wheel_base = 0.39`：その場旋回360°が床面テープ基準で一致することを確認済み
- 較正条件と異なる路面・速度では滑りが変わるため、円弧精度は別途実走確認が必要
- ROS側 `config.yaml` と同値に保つこと（`/params` で上書きされるため起動直後以外は一致する）

## 9. 移行手順

1. `Encoder` クラス新設（`lib/Encoder/`）＋単体動作確認（カウント方向・解像度）
2. `main.cpp` のISR・グローバル変数を `Encoder×2` に置換。`countsPerRev()=2048` への修正はここに含める（速度が2倍になるため要再確認）
3. 旧 `lib/Odometry/` ディレクトリ削除
4. 新 `Odometry` クラス新設（積算のみ、出力未接続）＋値の妥当性確認
5. `param_callback` に `odom.setWheelParams()` 追加
6. ESP側オドメトリとROS側 `/odom` の一致確認後、`nav_msgs/Odometry` 発行への切替を別設計で検討

## 10. 検証計画

- [ ] 1回転あたりカウント＝2048（X2：2逓倍）の実測確認（手回し＋シリアル表示）
- [ ] 左右方向符号の確認（前進時に両輪＋方向）
- [ ] `ros2 topic echo /vel` の値が従来の2倍になることの確認（既定値修正の効果）
- [ ] 長時間運転でのカウント飛び・オーバーフローなし（`int32_t` で約100万回転分、実用上問題なし）
- [ ] 既存 `mirs` 側pytestへの影響なし（FW変更のみ）
