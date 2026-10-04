# 0004: extra_packagesの個別リポジトリ化

- Status: 採用
- Date: 2026-09-14

## 背景

`extra_packages/`（`VoltageSensor`/`imu`/`SafetyEstop`）がgit管理外で、再現性がない。新機能を少しずつ乗せる方針には耐えない。

## 決定

- 各ディレクトリを独立したgitリポジトリにする（`git init`＋初回コミット済み）。
- 本体側の `.gitignore` は維持し、`lib_extra_dirs` で参照する。現運用（allowlist方式）と整合させる。
- GitHubにリモートを作り次第、submodule化する。それまでは各ディレクトリ内で個別コミットする。

## 結果

- 各libの変更履歴が追える。センサ追加＝新リポジトリ＋適合層＋登録1行。
- 代償：リポジトリが増える。横断変更時は複数repoのコミットが必要。
