# 非線形解析（動的解析）（RESP-D「07 非線形解析（動的解析）」）照合

**原典:** RESP-D 操作・計算マニュアル 計算編「07. 非線形解析（動的解析）」
（ユーザー提供資料、2026-07-12 照合）。本ドキュメントは同マニュアル「履歴特性」
および「立体解析モデルの非線形特性（既定の非線形特性）」との照合で追加・是正した
項目を記録する。

## 実装した項目

### 1. 履歴特性（履歴則）の拡充 — 逆行型・標準型・最大点指向型

**対象:** `squid-n-material/src/hysteresis.rs`（`HysteresisRule` / `HysteresisMaterial`）。

従来は武田型（`Takeda`）・原点指向型（`OriginOriented`）・スリップ型（`Slip`）のみで、
マニュアル「履歴特性」の以下の名前付き履歴則が欠落していた。

| 履歴則 | 追加 variant | 挙動（原典） |
|---|---|---|
| 逆行型 | `Retrograde` | 常にスケルトンカーブ上を進む（除荷・再載荷を可逆に辿り履歴ループを生じない） |
| 標準型 | `Standard` | 除荷は Masing 則（相似則）。除荷開始剛性=初期剛性 K1、除荷後の第2/第3剛性は骨格の剛性低下率と同様 |
| 最大点指向型 | `MaxPointOriented` | \|δ\|<δy1 は原点勾配 K1。降伏後は戻り点から反対側の最大経験変形点を直線で指向（Clough 系） |

- `Retrograde`: `evaluate` 冒頭で常に `Branch::Skeleton` に短絡。
- `Standard`: 反転時に `Branch::Masing` へ遷移。`Q(θ)=Qr − 2·sgn(θr−θ)·g(|θr−θ|/2)`
  （`g`=スケルトン力）で除荷開始勾配 `g'(0)=K1`、反射点（反対側 \|θ\|≥\|反転点\|）で
  スケルトンへ復帰する。
- `MaxPointOriented`: 反転時（降伏後）に `Branch::PeakOriented` へ遷移。戻り点から
  反対側の最大経験点への直線を辿り、到達でスケルトンへ復帰。

**検証:** `hysteresis.rs` 手計算照合テスト（逆行型のスケルトン可逆性、標準型の除荷開始
剛性=K1・反射点復帰・θ=0 で履歴枝上、最大点指向型のピーク指向補間、新規則の
スケルトン一致）。

### 2. 武田型の除荷剛性式を原典へ是正

**対象:** `squid-n-material/src/hysteresis.rs::unloading_stiffness`。

原典「履歴特性 武田型」は `Kd+ = K0·|δmax/δy2|^(−ν)`（K0=初期勾配、δy2=降伏変形、
ν=除荷指数）を規定するが、従来実装は基準を降伏割線 `Ky=My/θy` としていた（誤り）。
初期勾配 `K0 = Mc/θc` を基準に是正した（既存 `alpha` を指数 ν として流用）。
武田型ループのスナップショット（除荷ゼロ交差点）が是正値に更新される（骨格の折れ点は
不変）。

### 3. 既定の非線形特性表 → 材端曲げバネへの配線

**対象:** `squid-n-core`（`HysteresisModel`・`default_member_hysteresis`・
`Model::member_hysteresis_attrs`）、`squid-n-element/src/factory/mod.rs`、
`squid-n-edit`（`SetMemberHysteresis`）、`squid-n-app`（部材表・設計ビュー）。

原典「立体解析モデルの非線形特性」の既定表（梁の曲げ: RC/SRC/CFT=武田型、S=標準型）を
実装し、材端集中バネ（`ConcentratedSpringBeam`）へ配線した。従来は全ての梁が kinematic
バイリニアで、RC 梁が武田型（剛性低下トリリニア）にならない不整合があった。

- **core:** `HysteresisModel{Auto,Retrograde,Standard,OriginOriented,MaxPointOriented,Takeda}`、
  部材個別指定の側テーブル `member_hysteresis_attrs`（`isolator_attrs` と同方式）、
  既定表 `default_member_hysteresis(rc_like)`。
- **element/factory:** `resolve_member_hysteresis`（個別指定→断面形状による既定表）で
  履歴則を解決し、`build_flexural_springs` が武田型/逆行型/最大点指向型は
  `HysteresisMaterial`（トリリニア）、原点指向型はバイリニア、標準型は従来 kinematic
  バイリニアの材端バネを構築。トリリニア折れ点は Mc=0.56·Fc·Ze（RC）/My/3、
  My=規準の曲げ終局、θy=My/(αy·k_rot)（αy=0.3 既定）、Mu=1.1·My、θu=4·θy。
  武田型等の履歴材料は N-M 相関（`set_yield`）非対応のため相関はバイリニア時のみ適用。
- **edit/UI:** `SetMemberHysteresis`（undo 可）、部材表に「履歴則」列（自動/逆行型/
  標準型/原点指向型/最大点指向型/武田型。梁のみ）、設計タブに材端履歴則の集計表示。

**検証:** `factory/tests.rs`（既定表の解決・RC=武田型/S=標準型・個別指定 override・
RC 材端バネが実際に武田型の除荷剛性低下を示す実挙動テスト）、`edit/tests.rs`
（`SetMemberHysteresis` の undo/redo・不存在部材の Noop）、`core/model/tests.rs`
（既定表・側テーブル round-trip）。既存のプッシュオーバー/時刻歴/段階的耐力喪失
テストは全て緑（回帰なし）。

## 原典照合が必要な埋め込み値（技術リード確認用）

- 武田型トリリニアの材端骨格の簡略値: ひび割れ κ=0.56、降伏時剛性低下率 αy=0.3
  （既定）、終局倍率 1.1、塑性率 4。RESP-D 準拠の菅野 αy 精算はファイバ/skeleton 側
  （`build_rc_member_skeleton`）で行う方針であり、材端集中バネは簡略骨格を用いる。
- 武田型除荷指数 ν（`alpha`）既定 0.4。

## 未実装項目（今後の課題／資料スコープ外）

RESP-D「07 非線形解析（動的解析）」との照合で確認したが**なお未着手**の主な差異
（いずれも本照合の履歴特性スライスの範囲外。優先度順は別途）:

- **辻・山田モデル（β 混合硬化）・JFE LY トリリニア:** 名前付き履歴則だが、等方/移動
  硬化の配分（β）や製品固有トリリニアの詳細規則が大きく、別途対応。
- **減衰マトリクス:** モード別減衰（per-mode h_i）、接線剛性比例（α1 一定 / h1 一定）の
  瞬間剛性からの C 再構成、累積型/非累積型は未対応（剛性比例・Rayleigh のみ）。
- **免震支承材の装置別ひずみ依存モデル:** LRB 統一型 CKd(γ)/CQd(γ)、オイレス Tri-Linear、
  修正 H-D、高減衰ゴム系、錫プラグ、転がり支承、球面すべり、温度依存等（要素は
  バイリニア/摩擦の汎用モデルのみ）。
- **制振要素:** マクスウェル要素、各社ダンパー（SUB、アンボンドブレース、TRC、
  ユニットゴム、RDT、筒状流体/粘性体）の部材モデル。
- **質点系（串団子）モデル:** 等価せん断/曲げせん断型、P-δ、スウェイ・ロッキング、
  免震考慮モデル。
- **鉄骨大梁の座屈考慮履歴（局部/横/連成座屈の Mu 式）・その他（位相差入力、
  鉄骨梁端部の累積損傷度/レインフロー法）。**
