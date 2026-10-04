// Host-side unit tests for PoseEstimator (control_taskから抜き出した状態推定).
// Run: `pio test -e native`
// Estimator自体はArduino非依存。Encoder/DiffDrive経由の結合確認のため
// ArduinoスタブをこのTUに定義する。
#include <unity.h>
#include <cmath>
#include <cstdint>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Encoder.hpp"
#include "DiffDrive.hpp"
#include "PoseEstimator.hpp"

// ---------- Arduino stub definitions ----------
static int s_pin_level[40] = {0};
struct ISRReg { int pin = -1; void (*handler)(void *) = nullptr; void *arg = nullptr; int mode = 0; };
static ISRReg s_isr[8];
static int s_isr_n = 0;
static unsigned long g_micros = 0;

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
    for (int i = 0; i < 40; i++) s_pin_level[i] = LOW;
    for (int i = 0; i < 8; i++) s_isr[i] = ISRReg{};
}

static void sim_edge(uint8_t pin, int level) {
    int prev = s_pin_level[pin];
    s_pin_level[pin] = level;
    if (prev == level) {
        return;
    }
    g_micros += 1000;
    for (int i = 0; i < s_isr_n; i++) {
        if (s_isr[i].pin == pin) s_isr[i].handler(s_isr[i].arg);
    }
}

static void drive_forward(uint8_t pa, uint8_t pb, int cycles) {
    for (int i = 0; i < cycles; i++) {
        sim_edge(pb, HIGH); sim_edge(pa, HIGH);
        sim_edge(pb, LOW);  sim_edge(pa, LOW);
    }
}

static void drive_backward(uint8_t pa, uint8_t pb, int cycles) {
    for (int i = 0; i < cycles; i++) {
        sim_edge(pa, HIGH); sim_edge(pb, HIGH);
        sim_edge(pa, LOW);  sim_edge(pb, LOW);
    }
}

// ---------- Unity fixtures ----------
void setUp(void) { sim_reset(); }
void tearDown(void) {}

// 直進25周期×4=+100カウント相当を1ステップで投入
static void drive_straight(DiffDrive &dd, int cycles, int32_t &cl, int32_t &cr,
                           int64_t &dl, int64_t &dr) {
    drive_backward(13, 14, cycles);
    drive_forward(4, 5, cycles);
    dd.sample(cl, cr, dl, dr);
}

void test_straight_pose_and_velocity(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    PoseEstimator est(4096.0, 0.04, 0.38, false);
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    drive_straight(dd, 25, cl, cr, dl, dr);
    EstimatorImu imu;
    est.update(dl, dr, dd.distLeft(), dd.distRight(), imu, 0.015);
    const float e = (float)(100.0 / 4096.0 * 2.0 * M_PI * 0.04);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, e, est.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.theta());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, e / 0.015f, est.v());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, e / 0.015f, est.velL());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, e / 0.015f, est.velR());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.w());
}

void test_spin_theta(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    PoseEstimator est(4096.0, 0.04, 0.38, false);
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    drive_forward(13, 14, 25);  // 左は論理逆転（-100）
    drive_forward(4, 5, 25);    // 右は+100
    dd.sample(cl, cr, dl, dr);
    TEST_ASSERT_EQUAL_INT64(-100, dl);
    TEST_ASSERT_EQUAL_INT64(100, dr);
    EstimatorImu imu;
    est.update(dl, dr, dd.distLeft(), dd.distRight(), imu, 0.015);
    const double d = 100.0 / 4096.0 * 2.0 * M_PI * 0.04;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(2.0 * d / 0.38), est.theta());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.x());
}

void test_glitch_freezes_but_reference_advances(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    PoseEstimator est(4096.0, 0.04, 0.38, false);
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    drive_straight(dd, 25, cl, cr, dl, dr);
    EstimatorImu imu;
    est.update(dl, dr, dd.distLeft(), dd.distRight(), imu, 0.015);
    const float x0 = est.x();
    // 単発スパイクは棄却され、前回値が進むため次周期に跳躍を持ち越さない
    est.update(10000, 10000, 1.0, 1.0, imu, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, x0, est.x());
    drive_straight(dd, 25, cl, cr, dl, dr);
    est.update(dl, dr, dd.distLeft(), dd.distRight(), imu, 0.015);
    const float e = (float)(100.0 / 4096.0 * 2.0 * M_PI * 0.04);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, x0 + e, est.x());
}

void test_reset(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    PoseEstimator est(4096.0, 0.04, 0.38, false);
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    drive_straight(dd, 25, cl, cr, dl, dr);
    EstimatorImu imu;
    est.update(dl, dr, dd.distLeft(), dd.distRight(), imu, 0.015);
    est.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.theta());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.v());
}

void test_ekf_path_turns_with_gyro(void) {
    Encoder l(13, 14);
    Encoder r(4, 5);
    DiffDrive dd(l, r, true, false);
    dd.begin();
    dd.setWheelParams(0.04, 0.38);
    PoseEstimator est(4096.0, 0.04, 0.38, true);
    EstimatorImu imu;
    imu.imu_ok = true;
    imu.gz = 0.5f;
    imu.ax = 0.0f;
    imu.ay = 0.0f;
    imu.az = 9.80665f;
    est.update(0, 0, 0.0, 0.0, imu, 0.1);
    // ジャイロ0.5で0.1秒→thetaは正方向に振れる。
    // bias更新が交差共分散でthetaも補正するため、ちょうど0.05にはならない（正しいEKF動作）
    TEST_ASSERT_TRUE(est.theta() > 0.0f && est.theta() <= 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, est.x());
}

void test_apply_params_changes_base(void) {
    PoseEstimator est(4096.0, 0.04, 0.38, false);
    est.applyParams(0.04, 0.78);  // base2倍→同一差分でtheta半分
    const double d = 100.0 / 4096.0 * 2.0 * M_PI * 0.04;
    EstimatorImu imu;
    const double dist = 100.0 / 4096.0 * 2.0 * M_PI * 0.04;
    est.update(-100, 100, -dist, dist, imu, 0.015);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(2.0 * d / 0.78), est.theta());
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_straight_pose_and_velocity);
    RUN_TEST(test_spin_theta);
    RUN_TEST(test_glitch_freezes_but_reference_advances);
    RUN_TEST(test_reset);
    RUN_TEST(test_ekf_path_turns_with_gyro);
    RUN_TEST(test_apply_params_changes_base);
    return UNITY_END();
}
