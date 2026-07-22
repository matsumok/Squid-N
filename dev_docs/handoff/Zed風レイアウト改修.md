# Zed 風レイアウト改修（ドック化＋イベントログ）

作成日: 2026-07-22
対象ブランチ: `feature/zed-style-layout`
関連文書: `../specs/UI設計.md`（§1 を本改修に合わせて改訂済み）

## 1. 目的

GUI の全体構成を Zed エディタ風の「開閉可能ドック＋下部バーのトグル」型へ改修する。

- 下部ステータスバーのアイコンで左右・下のパネルを開閉できる（左端 🗂＝左ドック・📜＝下ドック、右端 🔍＝右ドック）。
- 中央下部に処理ログ（イベントログ）専用の領域を新設し、1行しか出せなかった
  `last_error`/`last_notice` を時系列の履歴として追えるようにする。

## 2. 実装内容

### 2.1 レイアウト機構の置き換え（Stage 1）

`crates/squid-n-app/src/app/mod.rs` の `impl eframe::App for App::ui`:

- `available_rect_before_wrap()` の手動矩形分割＋ `allocate_ui_at_rect`（deprecated）＋
  自前ドラッグハンドルを全廃し、egui 標準パネルの `show_inside` 連続呼び出しへ置換。
  呼び出し順は top（ツールバー）→ bottom（ステータスバー）→ left → right →
  bottom（ログドック）→ CentralPanel。下ドックを左右ドックの間（中央領域の下部）に
  出すため、左右より後に呼ぶ必要がある。
- egui 0.34.3 では `TopBottomPanel`/`SidePanel` および `default_width` 等が deprecated
  （統合 API `egui::Panel` へ移行）のため、`egui::Panel::top/bottom/left/right` と
  `default_size`/`size_range`/`exact_size` を使用（clippy `-D warnings` 対策でもある）。
- `App::left_panel_width` フィールドは廃止（egui パネルの標準リサイズが幅記憶を担う）。
- `App` に `left_dock_open`（初期 true）/`right_dock_open`（初期 true）/
  `bottom_dock_open`（初期 false）を追加（gui feature 限定）。
- `status_bar`（panels.rs）を `&mut self` 化し、ドック開閉トグルを追加。右ゾーンの
  確保幅はサマリ文字幅に 🔍 トグル分（アイコン幅＋ボタン余白）を加える必要がある
  （不足すると左ゾーンのエラー表示と重なる）。

### 2.2 イベントログ（Stage 2）

- `LogLevel`/`LogEntry`/`EventLog` を mod.rs に新設（cfg なし＝ヘッドレスでも保持）。
  上限 1000 件、時刻はアプリ起動からの経過時間（std のみではローカル壁時計の表記が
  できないため。表示「mm:ss」）。
- メッセージ発行を `App::report_error / report_notice / report_info`（actions.rs）へ一元化。
  crate 内の `last_error = Some(...)` / `last_notice = Some(...)` 直接代入は全て置換済み
  （残っているのは report_* 自身の実装 2 箇所のみ）。
- `report_error` は下ドックを自動で開く。ステータスバーの ⚠ 1行表示もクリックで
  下ドックが開く。
- 解析ジョブは開始時に「⏳ {label} を開始」、`poll_job` 成功時に「✅ {label} が完了
  ({:.1}s)」を Info でログ。失敗時は各 `apply_*` が report_error 経由になるため
  完了ログは出さない（二重ログ回避）。
- 単体テスト: `EventLog` の上限動作、`report_error` の last_error／ログ両反映
  （src/app/tests.rs）。

### 2.3 検証

CI 同条件＋ gui feature（非デフォルトのため `--workspace` では検証されない点に注意）:

- `cargo clippy --workspace --all-targets --locked -- -D warnings`
- `cargo clippy -p squid-n-app --features gui --all-targets -- -D warnings`
- `cargo test -p squid-n-app --features gui`（137 件）／ feature なし（96 件）
- `cargo fmt --all`

## 3. 残課題（次段階の候補）

1. **左ドックのパネル切替式化**: 現在はナビゲータ＋（モデル/荷重タブ時のみ）編集パネルの
   縦積み。Zed 同様に下部バーのアイコンごとに「ナビゲータ／編集テーブル／表示フィルタ」を
   切り替える形にすると、幅の狭いドックでも各機能が使いやすくなる。
2. **編集テーブルの下ドック移設**: 節点・部材等の横長テーブルは左ドックより下ドック
   （ログとタブ併置）の方が視認性が良い。3D 選択→該当行ハイライトの連動も自然になる。
3. **解析設定の右ドック移設**: 解析タブの縦長設定フォーム（panels.rs の
   `analysis_tab_panel`）を右ドックのパネルにすると、3D を見ながら設定変更できる。
4. **工程タブのプリセット化**: 工程タブを「各ドックのアクティブパネル＋中央表示の
   プリセット切替」に弱め、工程をまたぐ操作（結果を見ながら断面修正など）を楽にする。
5. **警告一覧タブ**: モデル整合性チェック結果を下ドックの別タブにし、クリックで該当
   部材へジャンプ（diagnostics 相当）。
6. ドックのドラッグ入替・中央のタブ分割まで狙う場合は `egui_dock` 導入を検討
   （固定ドック＋トグルだけなら標準パネルで足りる）。
