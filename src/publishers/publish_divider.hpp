#pragma once
#include <stdint.h>

/** @brief 呼出し回数分周器。DIV回に1回だけ真を返す（初回真はDIV回目の呼出し）。
 *  @details 15ms ros周期に対する発行間引き用。rcl/Arduino非依存のためhost試験可。
 *  dividerは1以上であること。 */
class PublishDivider {
public:
    explicit PublishDivider(uint8_t divider) : divider_(divider) {}

    /** @brief 1周期分進める。
     *  @return 今周期に発行すべきときtrue。 */
    bool tick() {
        ++count_;
        if (count_ < divider_) {
            return false;
        }
        count_ = 0;
        return true;
    }

    /** @brief 計数を捨てて次tickを1回目に戻す。 */
    void reset() { count_ = 0; }

private:
    uint8_t divider_;
    uint8_t count_ = 0;
};
