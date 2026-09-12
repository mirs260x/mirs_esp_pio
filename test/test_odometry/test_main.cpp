// Host-side unit tests for Encoder / DiffDrive / Odometry.
// Run: `pio test -e native` (Arduino API provided by test/mocks/Arduino.h,
// defined below in this single TU).
#include <unity.h>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Encoder.hpp"
#include "DiffDrive.hpp"
#include "Odometry.hpp"
#include "VelocityCalculator.hpp"

using namespace mirs2605;

// ---------- Arduino stub definitions ----------
static int s_pin_level[40] = {0};
struct ISRReg { int pin = -1; void (*handler)(void *) = nullptr; void *arg = nullptr; int mode = 0; };
static ISRReg s_isr[8];
static int s_isr_n = 0;

void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t pin) { return s_pin_level[pin]; }
int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterruptArg(uint8_t pin, void (*h)(void *), void *arg, int mode) {
    s_isr[s_isr_n++] = {pin, h, arg, mode};
}
void noInterrupts() {}
void interrupts() {}

static void sim_reset() {
    s_isr_n = 0;
    for (int i = 0; i < 40; i++) s_pin_level[i] = LOW;
    for (int i = 0; i < 8; i++) s_isr[i] = ISRReg{};
}

// Set pin level, then fire matching ISRs (RISING only on 0->1).
static void sim_edge(uint8_t pin, int level) {
    int prev = s_pin_level[pin];
    s_pin_level[pin] = level;
    for (int i = 0; i < s_isr_n; i++) {
        if (s_isr[i].pin != pin) continue;
        if (s_isr[i].mode == RISING && !(prev == LOW && level == HIGH)) continue;
        s_isr[i].handler(s_isr[i].arg);
    }
}

// One forward quadrature cycle (B leads): 00->01->11->10->00.
// X2（2逓倍）のため+2カウント。
static void drive_forward(uint8_t pa, uint8_t pb, int cycles) {
    for (int i = 0; i < cycles; i++) {
        sim_edge(pb, HIGH); sim_edge(pa, HIGH);
        sim_edge(pb, LOW);  sim_edge(pa, LOW);
    }
}

// One backward cycle: 00->10->11->01->00.
static void drive_backward(uint8_t pa, uint8_t pb, int cycles) {
    for (int i = 0; i < cycles; i++) {
        sim_edge(pa, HIGH); sim_edge(pb, HIGH);
        sim_edge(pa, LOW);  sim_edge(pb, LOW);
    }
}

// ---------- Unity fixtures ----------
void setUp(void) { sim_reset(); }
void tearDown(void) {}

// ---------- Encoder tests ----------
void test_counts_per_rev(void) {
    Encoder e(4, 5);
    TEST_ASSERT_EQUAL_UINT32(2048, e.countsPerRev());
}

void test_forward(void) {
    Encoder e(4, 5);
    e.begin();
    drive_forward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(20, e.getCount());
}

void test_backward(void) {
    Encoder e(4, 5);
    e.begin();
    drive_backward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(-20, e.getCount());
}

void test_pin_order_defines_direction(void) {
    // 素のEncoderはピン順で符号が反転する物理特性の確認。
    // 論理的な正逆の定義はDiffDriveのreverse指定で行う。
    Encoder normal(4, 5);
    Encoder swapped(5, 4);
    normal.begin();
    swapped.begin();
    drive_forward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(20, normal.getCount());
    TEST_ASSERT_EQUAL_INT32(-20, swapped.getCount());
}

void test_reset(void) {
    Encoder e(4, 5);
    e.begin();
    drive_forward(4, 5, 5);
    e.reset();
    TEST_ASSERT_EQUAL_INT32(0, e.getCount());
}

// ---------- DiffDrive tests ----------
// 左はミラー実装のため反転指定。ハード変更時はこの指定だけ変える。
void test_diff_distances(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    // 左+500 / 右+700カウント（反転補正後）。
    // 左の物理的正転はミラー操作のためピン操作は逆向きにする。
    drive_backward(13, 14, 250);
    drive_forward(4, 5, 350);
    dd.update();
    const double cpr = 2048.0;
    const double el = 500.0 / cpr * 2.0 * M_PI * 0.04;
    const double er = 700.0 / cpr * 2.0 * M_PI * 0.04;
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)el, (float)dd.distLeft());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)er, (float)dd.distRight());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.38f, (float)dd.wheelBase());
}

void test_diff_snapshot(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    // 左の物理的正転はミラー操作。補正後は両輪とも+に数えること
    drive_backward(13, 14, 10);
    drive_forward(4, 5, 10);
    int32_t cl = 0, cr = 0;
    dd.snapshot(cl, cr);
    TEST_ASSERT_EQUAL_INT32(20, cl);
    TEST_ASSERT_EQUAL_INT32(20, cr);
}

void test_diff_reversal_hidden_from_odometry(void) {
    // 反転の有無で符号が反転し、Odometry側の式は変えずに済むこと
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive plain(l, r, false, false);
    DiffDrive flipped(l, r, true, false);
    plain.begin();
    flipped.begin();
    drive_backward(13, 14, 10);
    drive_forward(4, 5, 10);
    plain.update();
    flipped.update();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, -(float)plain.distLeft(), (float)flipped.distLeft());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)plain.distRight(), (float)flipped.distRight());
    // Odometryは距離だけ見るため式は不変。反転ありで直進が直進と読めること
    Odometry o1, o2;
    o1.update(plain.distLeft(), plain.distRight(), 0.015);
    o2.update(flipped.distLeft(), flipped.distRight(), 0.015);
    TEST_ASSERT_TRUE(fabsf(o1.theta) > 1e-3f);  // 補正なしでは旋回と誤読
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o2.theta);  // 補正ありで直進
}

// ---------- Odometry tests (距離入力のみ。Encoder不要) ----------
// ---------- 計算層テスト（VelocityCalculatorは計算層。IF層に速度計算を持たせない） ----------
void test_velocity_is_computed_by_calculator(void) {
    // 計算層の責務：カウント→速度。IF層（DiffDrive）は距離まで。
    VelocityCalculator calc(2048.0, 0.04, 0.015);
    int32_t prev = 0;
    double v = calc.calculate(500, prev);
    const double expected = 500.0 / 2048.0 * 2.0 * M_PI * 0.04 / 0.015;
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)expected, (float)v);
    TEST_ASSERT_EQUAL_INT32(500, prev);  // 前回値は呼出側管理のまま
    // 半径変更が反映されること
    calc.setWheelRadius(0.08);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)(expected * 2.0), (float)calc.calculate(1000, prev));
}

void test_odom_straight(void) {
    Odometry odom;
    const double d = 2.0 * M_PI * 0.04;  // 1回転分
    odom.update(d, d, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)d, odom.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.theta);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(d / 0.015), odom.v_linear);
}

void test_odom_spin(void) {
    Odometry odom;
    const double d = 2.0 * M_PI * 0.04;
    odom.update(-d, d, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, odom.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(2.0 * d / 0.38), odom.theta);
}

void test_odom_arc_matches_exact_solution(void) {
    Odometry odom;
    // 左+500 / 右+700カウント相当の円弧1ステップ
    const double cpr = 2048.0;
    const double dl = 500.0 / cpr * 2.0 * M_PI * 0.04;
    const double dr = 700.0 / cpr * 2.0 * M_PI * 0.04;
    odom.update(dl, dr, 0.015);
    // 厳密解（等速円弧）：R=d/dth, dx=R*sin(dth), dy=R*(1-cos(dth))
    const double d = (dl + dr) / 2.0;
    const double dth = (dr - dl) / 0.38;
    const double R = d / dth;
    // 中点法の誤差はO(dth^3)≈1e-5。更新後theta方式なら約1e-4ずれる。
    TEST_ASSERT_FLOAT_WITHIN(5e-5f, (float)(R * sin(dth)), odom.x);
    TEST_ASSERT_FLOAT_WITHIN(5e-5f, (float)(R * (1.0 - cos(dth))), odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)dth, odom.theta);
}

void test_odom_theta_normalized(void) {
    Odometry odom;
    // 合計約6rad回して[-PI, PI]に収まること
    const double e = 0.02;
    for (int i = 0; i < 60; i++) {
        odom.update(-e, e, 0.015);
    }
    TEST_ASSERT_TRUE(fabsf(odom.theta) <= (float)M_PI + 1e-3f);
    odom.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.theta);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_counts_per_rev);
    RUN_TEST(test_forward);
    RUN_TEST(test_backward);
    RUN_TEST(test_pin_order_defines_direction);
    RUN_TEST(test_reset);
    RUN_TEST(test_diff_distances);
    RUN_TEST(test_diff_snapshot);
    RUN_TEST(test_diff_reversal_hidden_from_odometry);
    RUN_TEST(test_velocity_is_computed_by_calculator);
    RUN_TEST(test_odom_straight);
    RUN_TEST(test_odom_spin);
    RUN_TEST(test_odom_arc_matches_exact_solution);
    RUN_TEST(test_odom_theta_normalized);
    return UNITY_END();
}
