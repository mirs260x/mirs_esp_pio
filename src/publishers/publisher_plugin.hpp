#pragma once
#include <rcl/rcl.h>

#include <stdint.h>

#include "SystemContext.hpp"

/** @brief ros_task用PublisherプラグインIF。センサ側Registryと対称の枠組み。
 *  @details 追加・削除は ros_task.cpp の登録リスト1行。#if分岐は不要
 *  （データ不在時は各 publish() が無発行にする）。
 *  micro-ROS依存のため src/ 配置（host試験対象外）。各プラグインは薄く保つこと。 */
/** @brief warn_unused_result 対策。周期発行・後始末の成否は上位で扱わない。 */
inline void ignoreResult(rcl_ret_t rc) {
    (void)rc;
}

class IPublisherPlugin {public:
    virtual ~IPublisherPlugin() = default;
    /** @return ログ・識別用名 */
    virtual const char *name() const = 0;
    /** @brief publisher生成＋メッセージバッファ確保。
     *  @return true: 開始成功。false: ros_setup全体を失敗させる */
    virtual bool advertise(rcl_node_t *node) = 0;
    /** @brief 周期発行。データ不在時は無発行にすること。 */
    virtual void publish(const SharedMotion &motion, const SharedSensor &sensor,
                         int32_t now_sec, uint32_t now_nsec) = 0;
    /** @brief publisher破棄＋バッファ解放。未advertiseへの呼出しも安全にすること。 */
    virtual void release(rcl_node_t *node) = 0;
};
