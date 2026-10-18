#pragma once
#include "SystemContext.hpp"

/** @brief sensor_task用プラグインIF。
 *  @details begin()がfalseを返したプラグインはその1個だけ無効化され、他は継続する。
 *  update()は蓄積中のSharedSensorを受け取り、担当フィールドだけ上書きする。
 */
class ISensorPlugin {
public:
    virtual ~ISensorPlugin() = default;
    /** @return ログ表示用プラグイン名 */
    virtual const char *name() const = 0;
    /** @return true: 開始成功。false: 無効化（不在・故障） */
    virtual bool begin() = 0;
    /** @brief 担当フィールドをsnapshotに書き込む。
     *  @param snapshot 蓄積中のセンサスナップショット
     */
    virtual void update(SharedSensor &snapshot) = 0;
};
