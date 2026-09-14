// Host-side unit tests for RcReceiver / RobotController.
// Run: `pio test -e native` (Arduino API provided by test/mocks/Arduino.h,
// defined below in this single TU).
#include <unity.h>

#include "RcReceiver.hpp"
#include "RobotController.hpp"

// ---------- Arduino stub definitions (robot side) ----------
static int s_pin_level[40] = {0};
static unsigned long g_micros = 0;
static unsigned long g_millis = 0;
struct ISRReg { int pin = -1; void (*handler)(void *) = nullptr; void *arg = nullptr; int mode = 0; };
static ISRReg s_isr[16];
static int s_isr_n = 0;

void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t pin) { return s_pin_level[pin]; }
void digitalWrite(uint8_t, uint8_t) {}
int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterruptArg(uint8_t pin, void (*h)(void *), void *arg, int mode) {
    s_isr[s_isr_n++] = {pin, h, arg, mode};
}
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
void ledcWrite(uint8_t, uint32_t) {}
void noInterrupts() {}
void interrupts() {}
unsigned long micros() { return g_micros; }
unsigned long millis() { return g_millis; }

static void sim_reset() {
    s_isr_n = 0;
    g_micros = 0;
    g_millis = 0;
    for (int i = 0; i < 40; i++) s_pin_level[i] = LOW;
    for (int i = 0; i < 16; i++) s_isr[i] = ISRReg{};
}

// Set pin level, then fire matching CHANGE ISRs.
static void sim_set(uint8_t pin, int level) {
    int prev = s_pin_level[pin];
    s_pin_level[pin] = level;
    if (prev == level) return;
    for (int i = 0; i < s_isr_n; i++) {
        if (s_isr[i].pin == pin) s_isr[i].handler(s_isr[i].arg);
    }
}

// Drive one servo pulse of width_us on pin (rise at now, fall after width).
static void rc_pulse(uint8_t pin, uint32_t width_us) {
    sim_set(pin, HIGH);
    g_micros += width_us;
    g_millis += 1;
    sim_set(pin, LOW);
}

static const uint8_t P_LEFT = 21, P_MODE = 22, P_RIGHT = 23;
static const uint8_t PINS[3] = {P_LEFT, P_MODE, P_RIGHT};

// ---------- Unity fixtures ----------
void setUp(void) { sim_reset(); }
void tearDown(void) {}

// ---------- pulseToNormalized (pure function) ----------
void test_norm_neutral(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, RcReceiver::pulseToNormalized(1496));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, RcReceiver::pulseToNormalized(0));  // 未受信
}

void test_norm_endpoints(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, RcReceiver::pulseToNormalized(890));
    // MID=1496は中央(1495)より1us高いため上限は0.9983になる
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.9983f, RcReceiver::pulseToNormalized(2100));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.8248f, RcReceiver::pulseToNormalized(1995));
}

// ---------- MANUAL drive / stop ----------
void test_manual_drive(void) {
    RcReceiver rc;
    rc.begin(PINS, 3);
    RobotController ctrl(rc, 0.38, 0.8f, 1000);
    rc_pulse(P_LEFT, 1995);
    rc_pulse(P_MODE, 1496);
    rc_pulse(P_RIGHT, 1496);
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_EQUAL_INT(RobotController::MODE_MANUAL, ctrl.getControlMode());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.6598f, (float)ctrl.getLeftVelCmd());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, (float)ctrl.getRightVelCmd());
}

void test_manual_stops_without_rc(void) {
    RcReceiver rc;
    rc.begin(PINS, 3);
    RobotController ctrl(rc, 0.38, 0.8f, 1000);
    ctrl.update(0, 1, 2, 100);  // 一度もパルスなし
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, (float)ctrl.getLeftVelCmd());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, (float)ctrl.getRightVelCmd());
}

// ---------- mode toggle ----------
void test_toggle_to_ros2_and_back(void) {
    RcReceiver rc;
    rc.begin(PINS, 3);
    RobotController ctrl(rc, 0.38, 0.8f, 1000);
    rc_pulse(P_LEFT, 1496);
    rc_pulse(P_MODE, 1496);
    rc_pulse(P_RIGHT, 1496);
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_EQUAL_INT(RobotController::MODE_MANUAL, ctrl.getControlMode());

    rc_pulse(P_MODE, 1995);  // 立上り→ROS2
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_EQUAL_INT(RobotController::MODE_ROS2, ctrl.getControlMode());

    rc_pulse(P_MODE, 1496);  // 立下り→変化なし
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_EQUAL_INT(RobotController::MODE_ROS2, ctrl.getControlMode());

    rc_pulse(P_MODE, 1995);  // 立上り→MANUAL奪取
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_EQUAL_INT(RobotController::MODE_MANUAL, ctrl.getControlMode());
}

// ---------- ROS2 without RC (RC不要化) ----------
void test_ros2_continues_without_rc(void) {
    RcReceiver rc;
    rc.begin(PINS, 3);
    RobotController ctrl(rc, 0.38, 0.8f, 1000);
    ctrl.setControlMode(RobotController::MODE_ROS2);
    ctrl.updateRos2Command(0.5f, 0.0f);
    ctrl.update(0, 1, 2, 100);  // RC無効のまま
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.5f, (float)ctrl.getLeftVelCmd());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.5f, (float)ctrl.getRightVelCmd());
}

// ---------- watchdog ----------
void test_watchdog_stops(void) {
    RcReceiver rc;
    rc.begin(PINS, 3);
    RobotController ctrl(rc, 0.38, 0.8f, 1000);
    ctrl.setControlMode(RobotController::MODE_ROS2);
    ctrl.updateRos2Command(0.5f, 0.0f);
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.5f, (float)ctrl.getLeftVelCmd());

    g_millis = 1500;  // 1s以上途絶
    ctrl.update(0, 1, 2, 100);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, (float)ctrl.getLeftVelCmd());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, (float)ctrl.getRightVelCmd());
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_norm_neutral);
    RUN_TEST(test_norm_endpoints);
    RUN_TEST(test_manual_drive);
    RUN_TEST(test_manual_stops_without_rc);
    RUN_TEST(test_toggle_to_ros2_and_back);
    RUN_TEST(test_ros2_continues_without_rc);
    RUN_TEST(test_watchdog_stops);
    return UNITY_END();
}
