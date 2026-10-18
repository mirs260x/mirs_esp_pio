// Host-side unit tests for PIDController (dt-normalized form).
// Run: `pio test -e native`
// Spec: P = Kp*err, I = Ki*sum(err*dt), D = Kd*(err-prev)/dt.
// Gains are period-independent; dt is clamped to [5ms, 50ms].
// dt<=0 freezes integral/derivative state (P + held I only).
#include <unity.h>

#include "PIDController.hpp"

// ---------- Arduino stub definitions (PID uses no Arduino API) ----------
void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
void digitalWrite(uint8_t, uint8_t) {}
int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterruptArg(uint8_t, void (*)(void *), void *, int) {}
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
void ledcWrite(uint8_t, uint32_t) {}
void noInterrupts() {}
void interrupts() {}
unsigned long micros() { return 0; }
unsigned long millis() { return 0; }

void setUp(void) {}
void tearDown(void) {}

static constexpr double DT = 0.015;

// P項のみ: 80 * 0.1 = 8.0
void test_p_term(void) {
    PIDController pid(80.0, 0.0, 0.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 8.0, pid.compute(0.1, 0.0, DT));
}

// D項はdtで割る: 8 * 0.1 / 0.015 = 53.333...
void test_d_term_dt_normalized(void) {
    PIDController pid(0.0, 0.0, 8.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 8.0 * 0.1 / DT, pid.compute(0.1, 0.0, DT));
    // 2回目は差分0なのでD=0
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, pid.compute(0.1, 0.0, DT));
}

// I項はdt掛けの累積: 1回目 30*0.1*0.015=0.045、2回目 0.09
void test_i_term_dt_normalized(void) {
    PIDController pid(0.0, 30.0, 0.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.045, pid.compute(0.1, 0.0, DT));
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.09, pid.compute(0.1, 0.0, DT));
}

// 同じ物理時間なら周期が違ってもIの溜まりは等しい (0.03s分: 30*0.1*0.03=0.09)
void test_i_period_independent(void) {
    PIDController fast(0.0, 30.0, 0.0);
    for (int i = 0; i < 3; i++) {
        (void)fast.compute(0.1, 0.0, 0.01);
    }
    PIDController slow(0.0, 30.0, 0.0);
    (void)slow.compute(0.1, 0.0, 0.03);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.09, fast.getIntegralTerm() * 30.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.09, slow.getIntegralTerm() * 30.0);
}

// 出力は±255にclamp
void test_output_clamp(void) {
    PIDController pid(80.0, 0.0, 0.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 255.0, pid.compute(10.0, 0.0, DT));
    TEST_ASSERT_FLOAT_WITHIN(1e-9, -255.0, pid.compute(-10.0, 0.0, DT));
}

// dt<=0ではP+保持Iのみ、状態凍結 (D=0、積分・前回誤差不変)
void test_dt_invalid_freezes_state(void) {
    PIDController pid(0.0, 30.0, 8.0);
    (void)pid.compute(0.1, 0.0, DT);
    const double held = pid.getIntegralTerm();
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 30.0 * held, pid.compute(0.1, 0.0, 0.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-9, held, pid.getIntegralTerm());
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.1, pid.getPrevError());
}

// リセットで積分・前回誤差クリア (dt=0.03: P=30, I=100*0.5*0.03=1.5, D=0.9*0.5/0.03=15 → 46.5)
void test_reset(void) {
    PIDController pid(60.0, 100.0, 0.9);
    (void)pid.compute(0.5, 0.0, 0.03);
    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, pid.getIntegralTerm());
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, pid.getPrevError());
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 46.5, pid.compute(0.5, 0.0, 0.03));
}

// dtは公称(0.015s)の1/3〜3倍にクランプされる: 0.15s→0.045s扱い
void test_dt_relative_clamp(void) {
    PIDController pid(0.0, 0.0, 8.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 8.0 * 0.1 / 0.045, pid.compute(0.1, 0.0, 0.15));
}

// 公称周期を変えるとクランプ範囲も追従する
void test_set_nominal_dt(void) {
    PIDController pid(0.0, 0.0, 8.0);
    pid.setNominalDt(0.09);  // 範囲 0.03〜0.27
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 8.0 * 0.1 / 0.09, pid.compute(0.1, 0.0, 0.09));
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_p_term);
    RUN_TEST(test_d_term_dt_normalized);
    RUN_TEST(test_i_term_dt_normalized);
    RUN_TEST(test_i_period_independent);
    RUN_TEST(test_output_clamp);
    RUN_TEST(test_dt_invalid_freezes_state);
    RUN_TEST(test_reset);
    RUN_TEST(test_dt_relative_clamp);
    RUN_TEST(test_set_nominal_dt);
    return UNITY_END();
}
