# AGENTS.md — mirs_esp_pio

ESP32用micro-ROSファームウェア。モータ出力とフェイルセーフに関わる変更は、ROS 2側ではなくこのファームウェアが最終的に安全側へ遷移できることを維持する。

## 検証

```bash
pio test -e native
pio run
```

実機書き込み・モータ駆動は、明示的に依頼された場合だけ行う。

## 文書

- [README.md](README.md) はGitHub上で読むビルド・書き込み・運用マニュアルである。利用者に影響する変更では同時に更新する。
- 未完了項目は [TODO.md](TODO.md) に記録する。
- コンポーネント固有の設計判断は `docs/adr/`、コードコメントの基準は `docs/COMMENT_GUIDE.md` に従う。

## 注意

- `/cmd_vel`、操作元の調停、watchdog、RC信号喪失、E-Stop、電圧保護の変更は安全機能への影響を確認し、ホストテストを追加または更新する。
- `extra_packages/` は独立リポジトリを含むため、対象を限定して変更する。
