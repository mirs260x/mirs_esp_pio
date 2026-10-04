// Host-side unit tests for Encoder / DiffDrive / OdometryCalculator.
// Run: `pio test -e native` (Arduino API provided by test/mocks/Arduino.h,
// defined below in this single TU).
// Encoderはhostと同一実装（GPIO割込みX4自前計数）のため、計数ロジック自体を検証できる。
#include <unity.h>
#include <cmath>
#include <cstdint>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Encoder.hpp"
#include "DiffDrive.hpp"
#include "OdometryCalculator.hpp"
#include "VelocityCalculator.hpp"

// ---------- Arduino stub definitions ----------
static int s_pin_level[40] = {0};
struct ISRReg { int pin = -1; void (*handler)(void *) = nullptr; void *arg = nullptr; int mode = 0; };
static ISRReg s_isr[8];
static int s_isr_n = 0;
static unsigned long g_micros = 0;
static unsigned long g_step_us = 100;

void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t pin) { return s_pin_level[pin]; }
int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterruptArg(uint8_t pin, void (*h)(void *), void *arg, int mode) {
    s_isr[s_isr_n++] = {pin, h, arg, mode};
}
void noInterrupts() {}
void interrupts() {}
unsigned long micros() { return g_micros; }
unsigned long millis() { return 0; }

static void sim_reset() {
    s_isr_n = 0;
    g_micros = 0;
    g_step_us = 100;
    for (int i = 0; i < 40; i++) s_pin_level[i] = LOW;
    for (int i = 0; i < 8; i++) s_isr[i] = ISRReg{};
}

// Set pin level, then fire matching CHANGE ISRs (any level change fires).
// g_step_us だけ時刻を進める（既定100usで10usガードを通る）。
static void sim_edge(uint8_t pin, int level) {
    int prev = s_pin_level[pin];
    s_pin_level[pin] = level;
    if (prev == level) {
        return;
    }
    g_micros += g_step_us;
    for (int i = 0; i < s_isr_n; i++) {
        if (s_isr[i].pin != pin) continue;
        s_isr[i].handler(s_isr[i].arg);
    }
}

// One forward quadrature cycle (B leads): 00->01->11->10->00.
// X4（A/B両相の両エッジ）のため+4カウント。
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
    TEST_ASSERT_EQUAL_UINT32(4096, e.countsPerRev());
}

void test_forward(void) {
    Encoder e(4, 5);
    e.begin();
    drive_forward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(40, e.getCount());
}

void test_backward(void) {
    Encoder e(4, 5);
    e.begin();
    drive_backward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(-40, e.getCount());
}

void test_pin_order_defines_direction(void) {
    // 素のEncoderはピン順で符号が反転する物理特性の確認。
    // 論理的な正逆の定義はDiffDriveのreverse指定で行う。
    Encoder normal(4, 5);
    Encoder swapped(5, 4);
    normal.begin();
    swapped.begin();
    drive_forward(4, 5, 10);
    TEST_ASSERT_EQUAL_INT32(40, normal.getCount());
    TEST_ASSERT_EQUAL_INT32(-40, swapped.getCount());
}

void test_glitch_edges_ignored(void) {
    // 10us未満の連続エッジはノイズとして無視し、状態も進めないこと
    Encoder e(4, 5);
    e.begin();
    sim_edge(5, HIGH);  // 00→01: +1
    TEST_ASSERT_EQUAL_INT32(1, e.getCount());
    g_step_us = 5;
    sim_edge(4, HIGH);  // 01→11相当だが無視
    TEST_ASSERT_EQUAL_INT32(1, e.getCount());
    g_step_us = 100;
    sim_edge(4, LOW);  // 状態が01のままなので±0（進んでいれば-1になる）
    TEST_ASSERT_EQUAL_INT32(1, e.getCount());
    sim_edge(4, HIGH);  // 01→11: +1（計数再開）
    TEST_ASSERT_EQUAL_INT32(2, e.getCount());
}

void test_reset(void) {
    Encoder e(4, 5);
    e.begin();
    drive_forward(4, 5, 5);
    e.reset();
    TEST_ASSERT_EQUAL_INT32(0, e.getCount());
}

// ---------- DiffDrive tests ----------
// 左エンコーダは取付向きにより反転指定。ハード変更時はこの指定だけ変える。
void test_diff_distances(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    // 左+500 / 右+700カウント（反転補正後）。
    // 左の物理的正転は取付向きによりピン操作は逆向きにする。
    drive_backward(13, 14, 125);
    drive_forward(4, 5, 175);
    dd.update();
    const double cpr = 4096.0;
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
    drive_backward(13, 14, 5);
    drive_forward(4, 5, 5);
    int32_t cl = 0, cr = 0;
    dd.snapshot(cl, cr);
    TEST_ASSERT_EQUAL_INT32(20, cl);
    TEST_ASSERT_EQUAL_INT32(20, cr);
}

void test_diff_reversal_hidden_from_odometry(void) {
    // 反転の有無で符号が反転し、OdometryCalculator側の式は変えずに済むこと
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
    // OdometryCalculatorは距離だけ見るため式は不変。反転ありで直進が直進と読めること
    OdometryCalculator o1, o2;
    o1.update(plain.distLeft(), plain.distRight(), 0.015);
    o2.update(flipped.distLeft(), flipped.distRight(), 0.015);
    TEST_ASSERT_TRUE(fabsf(o1.theta) > 1e-3f);  // 補正なしでは旋回と誤読
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o2.theta);  // 補正ありで直進
}

// ---------- OdometryCalculator tests (距離入力のみ。Encoder不要) ----------
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
    OdometryCalculator odom;
    const double d = 2.0 * M_PI * 0.04;  // 1回転分
    odom.update(d, d, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)d, odom.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, odom.theta);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(d / 0.015), odom.v_linear);
}

void test_odom_spin(void) {
    OdometryCalculator odom;  // 既定base=0.39（config.yamlと同一）
    const double d = 2.0 * M_PI * 0.04;
    odom.update(-d, d, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, odom.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(2.0 * d / 0.39), odom.theta);
}

void test_odom_arc_matches_exact_solution(void) {
    OdometryCalculator odom;  // 既定base=0.39（config.yamlと同一）
    // 左+500 / 右+700カウント相当の円弧1ステップ
    const double cpr = 4096.0;
    const double dl = 500.0 / cpr * 2.0 * M_PI * 0.04;
    const double dr = 700.0 / cpr * 2.0 * M_PI * 0.04;
    odom.update(dl, dr, 0.015);
    // 厳密解（等速円弧）：R=d/dth, dx=R*sin(dth), dy=R*(1-cos(dth))
    const double d = (dl + dr) / 2.0;
    const double dth = (dr - dl) / 0.39;
    const double R = d / dth;
    // 中点法の誤差はO(dth^3)≈1e-5。更新後theta方式なら約1e-4ずれる。
    TEST_ASSERT_FLOAT_WITHIN(5e-5f, (float)(R * sin(dth)), odom.x);
    TEST_ASSERT_FLOAT_WITHIN(5e-5f, (float)(R * (1.0 - cos(dth))), odom.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)dth, odom.theta);
}

void test_odom_theta_normalized(void) {
    OdometryCalculator odom;
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

// ---------- int32ラップ対策（wrapDelta・int64累積） ----------
void test_wrap_delta_no_wrap(void) {
    TEST_ASSERT_EQUAL_INT64(11, VelocityCalculator::wrapDelta(1011, 1000));
    TEST_ASSERT_EQUAL_INT64(-11, VelocityCalculator::wrapDelta(1000, 1011));
}

void test_wrap_delta_across_int32(void) {
    // INT32_MAX付近での折返し：+11カウントが正しく求まること
    int32_t prev = INT32_MAX - 5;
    int32_t cur = INT32_MIN + 5;
    TEST_ASSERT_EQUAL_INT64(11, VelocityCalculator::wrapDelta(cur, prev));
    TEST_ASSERT_EQUAL_INT64(-11, VelocityCalculator::wrapDelta(prev, cur));
}

void test_wrap_delta_absorbs_hw16_wrap(void) {
    // 16bit幅カウンタ直読時の折返しに備えた畳み込み。
    // +32767→-32768を跨いだ前進6カウントは+6と解釈すること（逆走は-6）。
    TEST_ASSERT_EQUAL_INT64(6, VelocityCalculator::wrapDelta(-32765, 32765));
    TEST_ASSERT_EQUAL_INT64(-6, VelocityCalculator::wrapDelta(32765, -32765));
    // 通常の小差分は不変
    TEST_ASSERT_EQUAL_INT64(73, VelocityCalculator::wrapDelta(32840, 32767));
}

void test_velocity_int64_cumulative(void) {
    // 長時間運転のint64累積値でも正しく速度が出ること
    VelocityCalculator calc(2048.0, 0.04, 0.015);
    int64_t prev = 5000000000LL;
    double v = calc.calculate(prev + 500, prev);
    const double expected = 500.0 / 2048.0 * 2.0 * M_PI * 0.04 / 0.015;
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)expected, (float)v);
    TEST_ASSERT_EQUAL_INT64(5000000500LL, prev);
}

void test_velocity_int32_wrap(void) {
    // int32版もラップを吸収すること（符号オーバーフローUB回避）
    VelocityCalculator calc(2048.0, 0.04, 0.015);
    int32_t prev = INT32_MAX - 5;
    double v = calc.calculate(INT32_MIN + 5, prev);
    const double expected = 11.0 / 2048.0 * 2.0 * M_PI * 0.04 / 0.015;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)expected, (float)v);
    TEST_ASSERT_EQUAL_INT32(INT32_MIN + 5, prev);
}

void test_sample_returns_corrected_deltas(void) {
    // 本番経路：反転補正済みのカウント・差分・距離が一括で返ること
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    drive_backward(13, 14, 5);
    drive_forward(4, 5, 5);
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    dd.sample(cl, cr, dl, dr);
    TEST_ASSERT_EQUAL_INT32(20, cl);
    TEST_ASSERT_EQUAL_INT32(20, cr);
    TEST_ASSERT_EQUAL_INT64(20, dl);
    TEST_ASSERT_EQUAL_INT64(20, dr);
    const double e = 20.0 / 4096.0 * 2.0 * M_PI * 0.04;
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)e, (float)dd.distLeft());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, (float)e, (float)dd.distRight());
    // 無移動の2回目は差分ゼロ・カウント維持
    dd.sample(cl, cr, dl, dr);
    TEST_ASSERT_EQUAL_INT64(0, dl);
    TEST_ASSERT_EQUAL_INT64(0, dr);
    TEST_ASSERT_EQUAL_INT32(20, cl);
    TEST_ASSERT_EQUAL_INT32(20, cr);
}

void test_sample_delta_matches_count_accumulation(void) {
    // 差分の累積 == カウントの伸び（通常域）。呼び出し側の掛け直し不要の根拠
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    drive_backward(13, 14, 5);
    drive_forward(4, 5, 7);
    dd.sample(cl, cr, dl, dr);
    TEST_ASSERT_EQUAL_INT64(cl, dl);
    TEST_ASSERT_EQUAL_INT64(cr, dr);
    const int32_t first_l = cl, first_r = cr;
    drive_backward(13, 14, 3);
    drive_forward(4, 5, 2);
    dd.sample(cl, cr, dl, dr);
    TEST_ASSERT_EQUAL_INT64((int64_t)cl - first_l, dl);
    TEST_ASSERT_EQUAL_INT64((int64_t)cr - first_r, dr);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_counts_per_rev);
    RUN_TEST(test_forward);
    RUN_TEST(test_backward);
    RUN_TEST(test_pin_order_defines_direction);
    RUN_TEST(test_glitch_edges_ignored);
    RUN_TEST(test_reset);
    RUN_TEST(test_diff_distances);
    RUN_TEST(test_diff_snapshot);
    RUN_TEST(test_sample_returns_corrected_deltas);
    RUN_TEST(test_sample_delta_matches_count_accumulation);
    RUN_TEST(test_diff_reversal_hidden_from_odometry);
    RUN_TEST(test_velocity_is_computed_by_calculator);
    RUN_TEST(test_odom_straight);
    RUN_TEST(test_odom_spin);
    RUN_TEST(test_odom_arc_matches_exact_solution);
    RUN_TEST(test_odom_theta_normalized);
    RUN_TEST(test_wrap_delta_no_wrap);
    RUN_TEST(test_wrap_delta_across_int32);
    RUN_TEST(test_wrap_delta_absorbs_hw16_wrap);
    RUN_TEST(test_velocity_int64_cumulative);
    RUN_TEST(test_velocity_int32_wrap);
    return UNITY_END();
}
