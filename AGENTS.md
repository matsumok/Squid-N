# Squid-N

Rustで作成する日本の建築構造計算一貫プログラム
日本での利用を想定しており、コミットメッセージやコード中のコメント、プロンプトに対する応答などは日本語で行うこと。

## 開発コマンド

ビルド・テスト・静的解析・機能フラグ・ドキュメントのビルド手順は
[CONTRIBUTING.md](CONTRIBUTING.md) を参照すること。

特に、**コミット前には必ず静的解析（clippy / fmt）を CI と同条件で実行する**こと
（`--all-targets` がないとテストコードが clippy の対象外になる）。

```bash
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo fmt --all -- --check
```
