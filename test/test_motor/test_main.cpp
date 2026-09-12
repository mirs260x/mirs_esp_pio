// Host-side unit tests for MotorDriver / DiffMotors.
// Run: `pio test -e native` (Arduino API provided by test/mocks/Arduino.h,
// defined below in this single TU).
#include <unity.h>

#include "MotorDriver.hpp"
#include "DiffMotors.hpp"

using namespace mirs2605;

// ---------- Arduino stub definitions (motor side) ----------
static int s_dir_level[40];
static int s_pwm_duty[40];
static int s_attach_count = 0;

void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
void digitalWrite(uint8_t pin, uint8_t val) { s_dir_level[pin] = val; }
int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterruptArg(uint8_t, void (*)(void *), void *, int) {}
bool ledcAttach(uint8_t, uint32_t, uint8_t) { s_attach_count++; return true; }
void ledcWrite(uint8_t pin, uint32_t duty) { s_pwm_duty[pin] = (int)duty; }
void noInterrupts() {}
void interrupts() {}

static void sim_reset() {
    for (int i = 0; i < 40; i++) { s_dir_level[i] = -1; s_pwm_duty[i] = -1; }
    s_attach_count = 0;
}

// ---------- Unity fixtures ----------
void setUp(void) { sim_reset(); }
void tearDown(void) {}

// ---------- MotorDriver tests ----------
void test_duty_positive(void) {
    MotorDriver m(33, 32);
    m.begin();
    m.setDuty(100);
    TEST_ASSERT_EQUAL_INT(HIGH, s_dir_level[32]);
    TEST_ASSERT_EQUAL_INT(100, s_pwm_duty[33]);
}

void test_duty_negative(void) {
    MotorDriver m(33, 32);
    m.begin();
    m.setDuty(-100);
    TEST_ASSERT_EQUAL_INT(LOW, s_dir_level[32]);
    TEST_ASSERT_EQUAL_INT(100, s_pwm_duty[33]);
}

void test_duty_clamp(void) {
    MotorDriver m(33, 32);
    m.begin();
    m.setDuty(300);
    TEST_ASSERT_EQUAL_INT(255, s_pwm_duty[33]);
    m.setDuty(-300);
    TEST_ASSERT_EQUAL_INT(LOW, s_dir_level[32]);
    TEST_ASSERT_EQUAL_INT(255, s_pwm_duty[33]);
}

void test_stop(void) {
    MotorDriver m(33, 32);
    m.begin();
    m.setDuty(100);
    m.stop();
    TEST_ASSERT_EQUAL_INT(0, s_pwm_duty[33]);
}

void test_begin_attaches_pwm(void) {
    MotorDriver m(33, 32);
    m.begin();
    TEST_ASSERT_EQUAL_INT(1, s_attach_count);
}

// ---------- DiffMotors tests ----------
void test_pair_polarity(void) {
    // 既定（右反転）は現行MotorControllerのDIR論理と一致
    MotorDriver l(26, 25), r(33, 32);
    DiffMotors motors(l, r);
    motors.begin();
    motors.setBoth(100, 100);
    TEST_ASSERT_EQUAL_INT(HIGH, s_dir_level[25]);  // 左 正=HIGH
    TEST_ASSERT_EQUAL_INT(LOW, s_dir_level[32]);   // 右 正=LOW（反転）
    TEST_ASSERT_EQUAL_INT(100, s_pwm_duty[26]);
    TEST_ASSERT_EQUAL_INT(100, s_pwm_duty[33]);
}

void test_pair_no_polarity(void) {
    MotorDriver l(26, 25), r(33, 32);
    DiffMotors motors(l, r, false, false);
    motors.begin();
    motors.setBoth(100, 100);
    TEST_ASSERT_EQUAL_INT(HIGH, s_dir_level[25]);
    TEST_ASSERT_EQUAL_INT(HIGH, s_dir_level[32]);
}

void test_pair_stop(void) {
    MotorDriver l(26, 25), r(33, 32);
    DiffMotors motors(l, r);
    motors.begin();
    motors.setBoth(100, 100);
    motors.stop();
    TEST_ASSERT_EQUAL_INT(0, s_pwm_duty[26]);
    TEST_ASSERT_EQUAL_INT(0, s_pwm_duty[33]);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_duty_positive);
    RUN_TEST(test_duty_negative);
    RUN_TEST(test_duty_clamp);
    RUN_TEST(test_stop);
    RUN_TEST(test_begin_attaches_pwm);
    RUN_TEST(test_pair_polarity);
    RUN_TEST(test_pair_no_polarity);
    RUN_TEST(test_pair_stop);
    return UNITY_END();
}
