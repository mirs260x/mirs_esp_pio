# 0003: Doxygen軽量運用とコメント規約

- Status: 採用
- Date: 2026-09-14

## 背景

コメントに来歴・対話文脈（「旧〜」「現行」等）が混ざり、第三者に通じない。

## 決定

- Doxygenを採用する。ただし軽量形：公開ヘッダの `@brief`/`@param`/`@return` のみ。`.cpp` 内部は通常コメント。
- コードには timeless な仕様だけ書く。来歴・議論はコミット文・PR・ADRに書く。
- 物理量には単位必須。詳細は `docs/COMMENT_GUIDE.md`。
- `doxygen Doxyfile` で静的HTML＋XMLを `docs/generated/` に生成（git管理外、サーバ不要）。
- 警告ゼロを維持し、CIで強制する。

## 結果

- API文書とAI可読XMLが自動生成される。
- 代償：公開API追加時は3タグ必須という一手間。
