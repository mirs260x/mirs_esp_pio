#pragma once
#include <stddef.h>
#include <stdint.h>

/** @brief 固定容量のタスク内プラグインレジストリ。ヒープ不使用。
 *  @details 追加・削除は add() 1行。プラグイン同士は互いを知らない。
 *  begin失敗のプラグインは無効化され、他は継続する。
 *  divider指定で間引き周期（何制御周期に1回updateするか）をプラグイン毎に変えられる。
 *  @tparam Plugin update(Data&)/begin()/name() を持つプラグイン型
 *  @tparam Data プラグインに渡すスナップショット型
 *  @tparam Capacity 最大登録数
 */
template <typename Plugin, typename Data, size_t Capacity>
class Registry {
public:
    /** @brief プラグインを登録する。
     *  @param plugin 登録対象（寿命は呼び出し側が保証すること）
     *  @param divider 間引き周期。1=毎周期
     *  @return true: 登録成功。false: 満杯・null・divider=0
     */
    bool add(Plugin *plugin, uint32_t divider = 1) {
        if (plugin == nullptr || count_ >= Capacity || divider == 0) {
            return false;
        }
        entries_[count_] = Entry{plugin, divider, 0, false};
        ++count_;
        return true;
    }

    /** @return 登録数 */
    size_t size() const { return count_; }

    /** @param index 登録順序
     *  @return プラグイン名 */
    const char *name(size_t index) const { return entries_[index].plugin->name(); }

    /** @param index 登録順序
     *  @return begin成功済みか */
    bool isEnabled(size_t index) const { return entries_[index].enabled; }

    /** @brief プラグインを開始する。失敗時は無効化される。
     *  @param index 登録順序
     *  @return true: 開始成功
     */
    bool beginPlugin(size_t index) {
        Entry &entry = entries_[index];
        entry.enabled = entry.plugin->begin();
        return entry.enabled;
    }

    /** @brief 有効なプラグインのうち周期到来分を更新する。
     *  @param data プラグインに渡すスナップショット
     */
    void updateAll(Data &data) {
        for (size_t i = 0; i < count_; ++i) {
            Entry &entry = entries_[i];
            if (!entry.enabled) {
                continue;
            }
            if (++entry.counter < entry.divider) {
                continue;
            }
            entry.counter = 0;
            entry.plugin->update(data);
        }
    }

private:
    struct Entry {
        Plugin *plugin = nullptr;
        uint32_t divider = 1;
        uint32_t counter = 0;
        bool enabled = false;
    };
    Entry entries_[Capacity] = {};
    size_t count_ = 0;
};
