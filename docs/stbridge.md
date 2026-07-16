# ST-Bridge 連携（モデル入出力）

Squid-N は構造モデルを [ST-Bridge](https://www.building-smart.or.jp/meeting/buildingsmart/st-bridge/)（XML, 2.0 系）形式で**読み込み・書き出し**できる。他社の一貫計算プログラムや BIM ツールとモデルを受け渡すための入出力経路である。

実装は `squid-n-io` クレートの `stbridge` モジュール（`import_stbridge` / `export_stbridge`）にあり、GUI アプリ（`squid-n-app`）のファイルメニューから利用する。

## GUI からの読み込み・書き出し

ファイルメニューに以下の項目がある。

| メニュー | 動作 |
|---|---|
| 📥 **ST-Bridge 読込…** | `.stb`（または `.xml`）ファイルを選び、内部モデルへ取り込む。取り込んだモデルは検証（`validate`）を通ってから現在のモデルと差し替わる |
| 📤 **ST-Bridge 書出…** | 現在のモデルを ST-Bridge XML として `.stb` ファイルに書き出す |

- ファイル選択ダイアログの拡張子フィルタは `.stb` / `.xml`。
- ST-Bridge 読込は Squid-N ネイティブのプロジェクト（`.scz`）とは別系統であり、読み込んでもプロジェクトの保存先パスは設定されない（新規モデルとして開く扱い）。上書き保存するとネイティブの `.scz` として保存される。

## 対応バージョン

- **ST-Bridge 2.0 系のみ**を受け付ける（ルート要素 `ST_BRIDGE` の `version` 属性が `2.` で始まること）。
- 1.x 系や ST-Bridge でない XML は読み込みエラーになる。

## 対応範囲（意味的往復を保証するサブセット）

読み込み・書き出しの対象は、`import → export → 再 import` でモデルが**意味的に一致する**範囲に限定している。

| 分類 | 対象内容 |
|---|---|
| 節点 | 座標、所属層 |
| 層 | 名称、標高 |
| 材料 | ヤング係数 E、ポアソン比 ν、密度、コンクリート強度 Fc、鋼材強度 Fy |
| 断面 | 面積、断面二次モーメント（Iy・Iz）、ねじり定数 J、せい・幅などの物性 |
| 部材 | 柱（鉛直材）／大梁（水平材）、節点・断面・材料の参照、部材軸（`ref_vector`） |
| 荷重 | 荷重ケース（節点荷重） |

## 非対応（対象外）

以下は ST-Bridge 入出力の対象に含まれない。読み込み後は既定値になる。

- 解析結果・独自属性。
- 拘束条件（支点）・質量（ST-Bridge の幾何スコープ外）。
- 部材荷重・荷重組合せ。
- 床（スラブ）・ブレース・剛域・端部接合などの詳細。

### 断面表現に関する注意

断面は、実 ST-Bridge の形鋼ライブラリ参照（`StbSecColumn_S` 等）ではなく、内部モデルの物性をそのまま持つ独自要素 `StbSecRaw` として入出力する。これは「正準モデル（内部モデル）を唯一の真実とする」方針によるもので、Squid-N 同士の受け渡しでは物性が完全に往復する。一方、他社ソフトとの完全な相互運用には断面形状名のマッピングが必要であり、これは将来の課題である。

## ライブラリからの利用

`squid-n-io` を直接使う場合は次の関数を用いる。

```rust
use squid_n_io::stbridge::{import_stbridge, export_stbridge};

// 読み込み: ST-Bridge XML 文字列 → 内部モデル
let xml = std::fs::read_to_string("model.stb")?;
let model = import_stbridge(&xml)?;

// 書き出し: 内部モデル → ST-Bridge XML 文字列
let xml = export_stbridge(&model)?;
std::fs::write("model.stb", xml)?;
```
