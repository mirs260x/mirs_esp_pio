// Host-side unit tests for PublishDivider.
// Run: `pio test -e native`
// Spec: tick() returns true once every DIV calls (first true on the DIV-th call).
#include <unity.h>

#include "publish_divider.hpp"

void setUp(void) {}
void tearDown(void) {}

void test_divider_one_always_true(void) {
    PublishDivider d(1);
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_TRUE(d.tick());
    }
}

void test_divider_four_pattern(void) {
    PublishDivider d(4);
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_TRUE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_TRUE(d.tick());
}

void test_divider_reset_restarts_cycle(void) {
    PublishDivider d(4);
    d.tick();
    d.tick();
    d.reset();
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_FALSE(d.tick());
    TEST_ASSERT_TRUE(d.tick());
}

void test_divider_instances_independent(void) {
    PublishDivider a(2);
    PublishDivider b(4);
    TEST_ASSERT_FALSE(a.tick());
    TEST_ASSERT_FALSE(b.tick());
    TEST_ASSERT_TRUE(a.tick());
    TEST_ASSERT_FALSE(b.tick());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_divider_one_always_true);
    RUN_TEST(test_divider_four_pattern);
    RUN_TEST(test_divider_reset_restarts_cycle);
    RUN_TEST(test_divider_instances_independent);
    return UNITY_END();
}
