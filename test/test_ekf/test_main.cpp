// Host-side unit tests for Mirs2605Ekf.
// Run: `pio test -e native -f test_ekf`
#include <unity.h>
#include <cmath>

#include "Mirs2605Ekf.hpp"

void setUp(void) {}
void tearDown(void) {}

void test_initial_state_zero(void) {
    Mirs2605Ekf ekf;
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, ekf.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, ekf.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, ekf.theta());
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, ekf.v());
}

void test_predict_straight_uses_velocity(void) {
    Mirs2605Ekf ekf;
    ekf.updateVel(0.5f, 1e-6f);  // v を 0.5 m/s に引き寄せる
    for (int i = 0; i < 10; ++i) {
        ekf.predict(0.0f, 0.1f);  // dtは0.1sにクランプされるため10回で1秒分
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, ekf.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, ekf.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ekf.theta());
}

void test_predict_turn_uses_gyro_minus_bias(void) {
    Mirs2605Ekf ekf;
    for (int i = 0; i < 10; ++i) {
        ekf.predict(0.5f, 0.1f);  // 0.5 rad/s で合計1秒
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, ekf.theta());
}

void test_vel_update_pulls_velocity(void) {
    Mirs2605Ekf ekf;
    ekf.updateVel(1.0f);
    TEST_ASSERT_TRUE(ekf.v() > 0.5f && ekf.v() <= 1.0f);
}

void test_yaw_rate_update_estimates_bias(void) {
    Mirs2605Ekf ekf;
    // 真のバイアス +0.05 rad/s、真の角速度 0 → gz=0.05, w_enc=0
    for (int i = 0; i < 50; ++i) {
        ekf.updateYawRate(0.0f, 0.05f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, ekf.gyroBias());
}

void test_mag_update_corrects_theta(void) {
    Mirs2605Ekf ekf;
    for (int i = 0; i < 10; ++i) {
        ekf.predict(0.5f, 0.1f);  // theta=0.5 にずらす
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, ekf.theta());
    ekf.updateMagTheta(0.0f, 1e-4f);  // 強い地磁気観測で 0 に戻す
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, ekf.theta());
}

void test_mag_update_wraps_angle(void) {
    Mirs2605Ekf ekf;
    ekf.updateMagTheta(static_cast<float>(M_PI) - 0.1f, 1e-6f);
    ekf.updateMagTheta(-static_cast<float>(M_PI) + 0.1f, 1e-6f);
    TEST_ASSERT_TRUE(fabsf(ekf.theta()) <= static_cast<float>(M_PI) + 1e-3f);
}

void test_yaw_rate_gated_updates_at_low_speed(void) {
    Mirs2605Ekf ekf;
    for (int i = 0; i < 50; ++i) {
        ekf.updateYawRateGated(0.0f, 0.05f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, ekf.gyroBias());
}

void test_yaw_rate_gated_freezes_at_speed(void) {
    Mirs2605Ekf ekf;
    for (int i = 0; i < 50; ++i) {
        ekf.updateYawRateGated(0.0f, 0.05f, 0.5f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, ekf.gyroBias());
}

void test_zero_vel_freezes_motion(void) {    Mirs2605Ekf ekf;
    ekf.updateVel(0.5f, 1e-6f);
    ekf.updateZeroVel(1e-8f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, ekf.v());
    ekf.predict(0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, ekf.x());
}

void test_stationary_detector(void) {
    TEST_ASSERT_TRUE(Mirs2605Ekf::isStationary(0.0f, 0.0f, 9.80665f, 0.0f, 0.0f));
    TEST_ASSERT_FALSE(Mirs2605Ekf::isStationary(0.0f, 0.0f, 9.80665f, 0.5f, 0.0f));
    TEST_ASSERT_FALSE(Mirs2605Ekf::isStationary(0.0f, 0.0f, 9.80665f, 0.0f, 0.5f));
    TEST_ASSERT_FALSE(Mirs2605Ekf::isStationary(0.0f, 0.0f, 5.0f, 0.0f, 0.0f));
}

void test_mag_yaw_level_flight(void) {
    // 水平・北向き: mx>0, my=0 → yaw=0
    float yaw = Mirs2605Ekf::magYawTiltCompensated(50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.80665f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, yaw);
}

void test_normalize_angle(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Mirs2605Ekf::normalizeAngle(2.0f * static_cast<float>(M_PI)));
    // +PI/-PI 跨ぎの残差が短い側（+0.2）に回ること
    const float r = Mirs2605Ekf::angleResidual(-static_cast<float>(M_PI) + 0.1f,
                                              static_cast<float>(M_PI) - 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, r);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_zero);
    RUN_TEST(test_predict_straight_uses_velocity);
    RUN_TEST(test_predict_turn_uses_gyro_minus_bias);
    RUN_TEST(test_vel_update_pulls_velocity);
    RUN_TEST(test_yaw_rate_update_estimates_bias);
    RUN_TEST(test_yaw_rate_gated_updates_at_low_speed);
    RUN_TEST(test_yaw_rate_gated_freezes_at_speed);
    RUN_TEST(test_mag_update_corrects_theta);
    RUN_TEST(test_mag_update_wraps_angle);
    RUN_TEST(test_zero_vel_freezes_motion);
    RUN_TEST(test_stationary_detector);
    RUN_TEST(test_mag_yaw_level_flight);
    RUN_TEST(test_normalize_angle);
    return UNITY_END();
}
