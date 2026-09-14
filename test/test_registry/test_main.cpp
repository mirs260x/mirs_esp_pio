// Host-side unit tests for Registry (task-level plugin framework).
// Run: `pio test -e native`
#include <unity.h>

#include "Registry.hpp"

struct FakeData {
    int sum = 0;
};

struct FakePlugin {
    const char *plugin_name;
    bool begin_result;
    int begin_calls = 0;
    int update_calls = 0;
    int add_value = 1;
    const char *name() const { return plugin_name; }
    bool begin() {
        ++begin_calls;
        return begin_result;
    }
    void update(FakeData &data) {
        ++update_calls;
        data.sum += add_value;
    }
};

void setUp(void) {}
void tearDown(void) {}

// 登録と基本取得
void test_add_and_size(void) {
    Registry<FakePlugin, FakeData, 2> reg;
    FakePlugin a{"a", true}, b{"b", true};
    TEST_ASSERT_TRUE(reg.add(&a));
    TEST_ASSERT_TRUE(reg.add(&b));
    TEST_ASSERT_EQUAL_size_t(2, reg.size());
    TEST_ASSERT_EQUAL_STRING("a", reg.name(0));
}

// ガード: null・満杯・divider=0は拒否
void test_add_guards(void) {
    Registry<FakePlugin, FakeData, 1> reg;
    FakePlugin a{"a", true};
    TEST_ASSERT_FALSE(reg.add(nullptr));
    TEST_ASSERT_FALSE(reg.add(&a, 0));
    TEST_ASSERT_TRUE(reg.add(&a));
    FakePlugin b{"b", true};
    TEST_ASSERT_FALSE(reg.add(&b));
}

// begin失敗プラグインは無効化され、updateされない
void test_begin_failure_disables(void) {
    Registry<FakePlugin, FakeData, 2> reg;
    FakePlugin ok{"ok", true}, ng{"ng", false};
    (void)reg.add(&ok);
    (void)reg.add(&ng);
    TEST_ASSERT_TRUE(reg.beginPlugin(0));
    TEST_ASSERT_FALSE(reg.beginPlugin(1));
    TEST_ASSERT_TRUE(reg.isEnabled(0));
    TEST_ASSERT_FALSE(reg.isEnabled(1));
    FakeData data;
    reg.updateAll(data);
    TEST_ASSERT_EQUAL_INT(1, ok.update_calls);
    TEST_ASSERT_EQUAL_INT(0, ng.update_calls);
    TEST_ASSERT_EQUAL_INT(1, data.sum);
}

// 間引き: divider=4は4周期に1回だけupdateされる
void test_divider(void) {
    Registry<FakePlugin, FakeData, 1> reg;
    FakePlugin a{"a", true};
    (void)reg.add(&a, 4);
    (void)reg.beginPlugin(0);
    FakeData data;
    for (int i = 0; i < 8; ++i) {
        reg.updateAll(data);
    }
    TEST_ASSERT_EQUAL_INT(2, a.update_calls);
    TEST_ASSERT_EQUAL_INT(2, data.sum);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_add_and_size);
    RUN_TEST(test_add_guards);
    RUN_TEST(test_begin_failure_disables);
    RUN_TEST(test_divider);
    return UNITY_END();
}
