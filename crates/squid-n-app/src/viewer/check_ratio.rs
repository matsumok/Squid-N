//! 検定比図（部材検定・節点検定の検定比による着色）の描画。
//!
//! 検定表（`design_view.rs`）と同じ [`theme::status_color`] の3色規約
//! （≤0.8 緑=OK／≤1.0 黄=注意／>1.0 赤=NG）で部材・節点を着色し、
//! 検定表と3Dビューの見え方を一貫させる。連続的なコンター配色は採らない
//! （判定基準 0.8/1.0 との対応を優先するため。詳細は dev_docs の申し送りを参照）。
//!
//! 着色対象は [`CheckRatioFilter`]（最大＝全式の max、または特定の検定式のみ）で
//! 切り替えられ、部材内の検定位置ごとに正方形マーカーを重ねる「位置別マーカー」、
//! ホバー時に位置×式の内訳を見せるツールチップも提供する。

use std::collections::HashMap;

use crate::app::App;
use crate::theme;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_design_jp::{CheckComponent, CheckKind, CheckResult};

use super::CheckRatioFilter;

/// `CheckKind` の定義順（Bending→Shear→Bond→AxialBending→Axial→Deflection）で
/// 固定した全種一覧。フィルタ選択肢・ツールチップの列順を安定させるために使う。
const ALL_KINDS: [CheckKind; 6] = [
    CheckKind::Bending,
    CheckKind::Shear,
    CheckKind::Bond,
    CheckKind::AxialBending,
    CheckKind::Axial,
    CheckKind::Deflection,
];

/// フィルタ `filter` を `cr` に適用した結果（検定比, OK か）を返す（純粋関数）。
///
/// - `Max`: `cr.ratio`／`cr.ok` をそのまま返す（従来動作）。
/// - `Kind(k)`: `cr.components` から `kind == k` の最大検定比を探し
///   `Some((r, r <= 1.0))` を返す。該当する式が無ければ `None`
///   （＝この検定位置は当該式の検定対象外。着色・マーカーとも描かない）。
pub(super) fn ratio_for_filter(cr: &CheckResult, filter: CheckRatioFilter) -> Option<(f64, bool)> {
    match filter {
        CheckRatioFilter::Max => Some((cr.ratio, cr.ok)),
        CheckRatioFilter::Kind(k) => {
            let max_ratio = cr
                .components
                .iter()
                .filter(|c| c.kind == k)
                .map(|c| c.ratio)
                .fold(None, |acc: Option<f64>, r| {
                    Some(acc.map_or(r, |a| a.max(r)))
                });
            max_ratio.map(|r| (r, r <= 1.0))
        }
    }
}

/// `components` 中の最大検定比を与える `kind`（空なら `None`）。
fn dominant_kind_of(components: &[CheckComponent]) -> Option<CheckKind> {
    components
        .iter()
        .max_by(|a, b| a.ratio.partial_cmp(&b.ratio).unwrap())
        .map(|c| c.kind)
}

/// `cr.components` 中の最大検定比を与える支配式（空なら `None`）。
pub(super) fn dominant_kind(cr: &CheckResult) -> Option<CheckKind> {
    dominant_kind_of(&cr.components)
}

/// 与えられた検定結果群の `components` に実際に現れる `CheckKind` を、定義順で
/// 重複なく返す（純粋関数）。ツールバーの検定式フィルタ選択肢・ツールチップの
/// 列を「結果に現れる式だけ」に絞るために使う（RC モデルで「軸」等の無関係な
/// 選択肢が並ばないようにする）。
pub(super) fn available_check_kinds<'a, I>(components_iter: I) -> Vec<CheckKind>
where
    I: IntoIterator<Item = &'a [CheckComponent]>,
{
    let mut present = [false; ALL_KINDS.len()];
    for comps in components_iter {
        for c in comps {
            if let Some(idx) = ALL_KINDS.iter().position(|k| *k == c.kind) {
                present[idx] = true;
            }
        }
    }
    ALL_KINDS
        .iter()
        .copied()
        .zip(present)
        .filter_map(|(k, p)| p.then_some(k))
        .collect()
}

/// 部材（または節点）ごとに、フィルタ適用後の検定比・OK フラグを集計する
/// （純粋関数）。`items` は `(キー, フィルタ適用後の (検定比, OK) または None)`。
/// `None`（フィルタ対象外の位置）は無視され、対象位置が一つも無い部材・節点は
/// 集計結果に含まれない（＝未検定として扱われ、着色されない）。
fn max_ratio_by_key<K, I>(items: I) -> HashMap<K, (f64, bool)>
where
    K: Eq + std::hash::Hash,
    I: IntoIterator<Item = (K, Option<(f64, bool)>)>,
{
    let mut map: HashMap<K, (f64, bool)> = HashMap::new();
    for (key, val) in items {
        let Some((ratio, ok)) = val else {
            continue;
        };
        let entry = map.entry(key).or_insert((0.0_f64, true));
        if ratio > entry.0 {
            entry.0 = ratio;
        }
        entry.1 &= ok;
    }
    map
}

/// 部材ごとの（フィルタ適用後の）最大検定比・OK フラグを集計する。
fn max_ratio_by_elem<I: IntoIterator<Item = (ElemId, Option<(f64, bool)>)>>(
    items: I,
) -> HashMap<ElemId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 節点ごとの（フィルタ適用後の）最大検定比・OK フラグを集計する。
fn max_ratio_by_node<I: IntoIterator<Item = (NodeId, Option<(f64, bool)>)>>(
    items: I,
) -> HashMap<NodeId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 部材中点ラベルの文字列を組み立てる（純粋関数）。支配式が分かる場合
/// （フィルタ=最大かつ components が非空）は「1.13 せん断」のように併記し、
/// それ以外（フィルタ=特定式、または components が空の部材）は数値のみ。
pub(super) fn mid_label_text(ratio: f64, dominant: Option<CheckKind>) -> String {
    match dominant {
        Some(k) => format!("{:.2} {}", ratio, k.label()),
        None => format!("{:.2}", ratio),
    }
}

/// 部材 `elem_id` の全検定位置を `(xi, ok, components)` として抽出する
/// （純粋関数。ホバー詳細ツールチップの表データ生成に使う）。
pub(super) fn elem_check_positions(
    checks: &[(ElemId, f64, CheckResult)],
    elem_id: ElemId,
) -> Vec<(f64, bool, Vec<CheckComponent>)> {
    checks
        .iter()
        .filter(|(id, _, _)| *id == elem_id)
        .map(|(_, xi, cr)| (*xi, cr.ok, cr.components.clone()))
        .collect()
}

/// ホバー詳細ツールチップの1行分（1検定位置）のデータ。
pub(super) struct TooltipRow {
    /// 検定位置 xi ∈ [0,1]
    pub xi: f64,
    /// 列（`kinds`）に対応する検定比。該当式が無い列は `None`。
    pub values: Vec<Option<f64>>,
    pub ok: bool,
}

/// 部材1本分の「位置×式」ツールチップ表データを生成する（純粋関数）。
/// `positions` は当該部材の全検定位置 `(xi, ok, components)`
/// （[`elem_check_positions`] の戻り値）。
///
/// 戻り値は `(列に出す式の集合＝出現順の CheckKind, 各行データ)`。
pub(super) fn build_tooltip_rows(
    positions: &[(f64, bool, Vec<CheckComponent>)],
) -> (Vec<CheckKind>, Vec<TooltipRow>) {
    let kinds = available_check_kinds(positions.iter().map(|(_, _, c)| c.as_slice()));
    let rows = positions
        .iter()
        .map(|(xi, ok, comps)| {
            let values = kinds
                .iter()
                .map(|k| {
                    comps
                        .iter()
                        .filter(|c| c.kind == *k)
                        .map(|c| c.ratio)
                        .fold(None, |acc: Option<f64>, r| {
                            Some(acc.map_or(r, |a| a.max(r)))
                        })
                })
                .collect();
            TooltipRow {
                xi: *xi,
                values,
                ok: *ok,
            }
        })
        .collect();
    (kinds, rows)
}

/// 検定比図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.model.nodes` と同じ順序）。
pub(super) fn draw_check_ratio(painter: &egui::Painter, app: &App, pts: &[[f32; 2]]) {
    let Some(results) = &app.results else {
        draw_no_result_legend(painter);
        return;
    };
    // 部材検定・節点検定のどちらかがあれば描画する（耐震壁のみのモデル等では
    // 部材検定が空でも節点検定だけが存在しうる）。
    if results.checks.is_empty() && results.joint_checks.is_empty() {
        draw_no_result_legend(painter);
        return;
    }

    let filter = app.check_ratio_filter;
    let markers = app.check_ratio_markers;

    let elem_ratios = max_ratio_by_elem(
        results
            .checks
            .iter()
            .map(|(id, _xi, cr)| (*id, ratio_for_filter(cr, filter))),
    );
    let node_ratios = max_ratio_by_node(
        results
            .joint_checks
            .iter()
            .map(|(id, _label, cr)| (*id, ratio_for_filter(cr, filter))),
    );

    // 部材ごとの検定位置一覧（B-2 位置別マーカー・B-4 支配式ラベル用）。
    let mut checks_by_elem: HashMap<ElemId, Vec<(f64, &CheckResult)>> = HashMap::new();
    for (id, xi, cr) in &results.checks {
        checks_by_elem.entry(*id).or_default().push((*xi, cr));
    }

    // --- 部材の着色 ---
    for elem in &app.model.elements {
        let Some(&(ratio, ok)) = elem_ratios.get(&elem.id) else {
            continue;
        };
        let color = theme::status_color(ratio);

        // 壁（面要素）: 半透明ポリゴンで塗り、輪郭を検定比の色で強調する
        if elem.kind == squid_n_core::model::ElementKind::Wall && elem.nodes.len() >= 3 {
            let poly: Vec<egui::Pos2> = elem
                .nodes
                .iter()
                .filter_map(|n| {
                    let idx = n.index();
                    (idx < pts.len()).then(|| egui::pos2(pts[idx][0], pts[idx][1]))
                })
                .collect();
            if poly.len() == elem.nodes.len() {
                painter.add(egui::Shape::convex_polygon(
                    poly,
                    theme::translucent(color, 70),
                    egui::Stroke::new(2.0_f32, color),
                ));
            }
            continue;
        }

        // 線材: 両端を結ぶ線を検定比の色で描き、中点に数値ラベルを添える。
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let p0 = egui::pos2(pts[n0][0], pts[n0][1]);
        let p1 = egui::pos2(pts[n1][0], pts[n1][1]);
        // NG 部材は太さで目立たせる
        let width = if ok { 4.0_f32 } else { 5.0_f32 };
        painter.line_segment([p0, p1], egui::Stroke::new(width, color));

        let positions: &[(f64, &CheckResult)] = checks_by_elem
            .get(&elem.id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);

        // B-2: 位置別マーカー（検定位置ごとに正方形。フィルタ対象外の位置は描かない）。
        if markers {
            for &(xi, cr) in positions {
                let Some((r, _)) = ratio_for_filter(cr, filter) else {
                    continue;
                };
                let mx = p0.x + (p1.x - p0.x) * xi as f32;
                let my = p0.y + (p1.y - p0.y) * xi as f32;
                let mcolor = theme::status_color(r);
                const MARK: f32 = 7.0;
                let mrect =
                    egui::Rect::from_center_size(egui::pos2(mx, my), egui::vec2(MARK, MARK));
                painter.rect_filled(mrect, 0.0, mcolor);
                painter.rect_stroke(
                    mrect,
                    0.0,
                    egui::Stroke::new(1.0_f32, theme::WHITE),
                    egui::StrokeKind::Middle,
                );
                // NG の位置のみ数値ラベルを添える（全位置に出すと過密になるため）。
                if r > 1.0 {
                    painter.text(
                        egui::pos2(mrect.max.x + 2.0, mrect.min.y),
                        egui::Align2::LEFT_BOTTOM,
                        format!("{:.2}", r),
                        egui::FontId::proportional(10.0),
                        theme::PARETO_RED,
                    );
                }
            }
        }

        // B-4: 中点ラベル（部材内最大＝ratio）。フィルタ=最大のときは支配式を併記する。
        let dominant = if filter == CheckRatioFilter::Max {
            positions
                .iter()
                .max_by(|a, b| a.1.ratio.partial_cmp(&b.1.ratio).unwrap())
                .and_then(|(_, cr)| dominant_kind(cr))
        } else {
            None
        };
        let mid = egui::pos2((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
        let (font_size, label_color) = if ok {
            (11.0, theme::GRAY_700)
        } else {
            // NG はフォントを大きくし赤字で目立たせる
            (12.0, theme::PARETO_RED)
        };
        painter.text(
            mid,
            egui::Align2::CENTER_BOTTOM,
            mid_label_text(ratio, dominant),
            egui::FontId::proportional(font_size),
            label_color,
        );
    }

    // --- 節点検定（接合部・パネルゾーン・耐震壁など）の表示 ---
    // NodeId の内部値はそのまま配列添字とは限らないため、`app.model.nodes` を
    // 走査してインデックスを求め（`enumerate` の添字が実際の `pts` の添字）、
    // `node.id` と突き合わせてから `pts` を引く。
    for (idx, node) in app.model.nodes.iter().enumerate() {
        let Some(&(ratio, _ok)) = node_ratios.get(&node.id) else {
            continue;
        };
        if idx >= pts.len() {
            continue;
        }
        let p = egui::pos2(pts[idx][0], pts[idx][1]);
        let color = theme::status_color(ratio);
        painter.circle_filled(p, 5.0, color);
        painter.circle_stroke(p, 5.0, egui::Stroke::new(1.0_f32, theme::VIEW_BG));
    }

    draw_legend(painter, app, &elem_ratios, &node_ratios, filter, markers);
}

/// B-3: 部材 `elem_id` の検定詳細（位置×式）をポインタ位置にツールチップ表示する。
/// `app.results.checks` に当該部材の検定が無ければ何も描かない。
pub(super) fn show_check_tooltip(ui: &egui::Ui, app: &App, elem_id: ElemId) {
    let Some(results) = &app.results else {
        return;
    };
    let positions = elem_check_positions(&results.checks, elem_id);
    if positions.is_empty() {
        return;
    }
    let basis = results
        .checks
        .iter()
        .find(|(id, _, _)| *id == elem_id)
        .map(|(_, _, cr)| cr.basis.clone())
        .unwrap_or_default();
    let (kinds, rows) = build_tooltip_rows(&positions);

    // `show_tooltip_at_pointer` は egui 0.34 で非推奨（`Tooltip` 型を使う新 API へ
    // 移行中）だが、ウィジェットに紐付かない任意位置へのツールチップ表示という
    // 用途には他に簡潔な代替が無いため、既存コード（app/panels.rs）と同じ方針で
    // `#[allow(deprecated)]` を付けて使用する。
    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("check_ratio_tooltip"),
        |ui| {
            ui.label(format!("部材 #{} ({basis})", elem_id.0));
            egui::Grid::new("check_ratio_tooltip_grid")
                .striped(true)
                .show(ui, |ui| {
                    ui.label("位置");
                    for k in &kinds {
                        ui.label(k.label());
                    }
                    ui.label("判定");
                    ui.end_row();
                    for row in &rows {
                        ui.label(format!("{:.2}", row.xi));
                        for v in &row.values {
                            match v {
                                Some(r) => {
                                    ui.colored_label(theme::status_color(*r), format!("{r:.2}"));
                                }
                                None => {
                                    ui.label("-");
                                }
                            }
                        }
                        if row.ok {
                            ui.label("OK");
                        } else {
                            ui.colored_label(theme::PARETO_RED, "NG");
                        }
                        ui.end_row();
                    }
                });
        },
    );
}

/// 検定結果が無い場合の案内表示。
fn draw_no_result_legend(painter: &egui::Painter) {
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        "検定結果がありません。解析タブから静的解析を実行してください。",
        egui::FontId::proportional(14.0),
        theme::GRAY_600,
    );
}

/// 検定式フィルタの表示名（凡例タイトル用）。
fn filter_label(filter: CheckRatioFilter) -> &'static str {
    match filter {
        CheckRatioFilter::Max => "最大",
        CheckRatioFilter::Kind(k) => k.label(),
    }
}

/// ビュー左上に検定比図の凡例（対象・最大値・NG件数・色見本・陳腐化注記）を描く。
#[allow(clippy::too_many_arguments)]
fn draw_legend(
    painter: &egui::Painter,
    app: &App,
    elem_ratios: &HashMap<ElemId, (f64, bool)>,
    node_ratios: &HashMap<NodeId, (f64, bool)>,
    filter: CheckRatioFilter,
    markers: bool,
) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 10.0;

    let max_ratio = elem_ratios
        .values()
        .chain(node_ratios.values())
        .map(|&(r, _)| r)
        .fold(0.0_f64, f64::max);
    let ng_count = elem_ratios
        .values()
        .chain(node_ratios.values())
        .filter(|&&(_, ok)| !ok)
        .count();

    let title_rect = painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        format!(
            "検定比図 (対象: {}, max={:.2}, NG {}件)",
            filter_label(filter),
            max_ratio,
            ng_count
        ),
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
    y = title_rect.max.y + 4.0;

    // 色見本: ≤0.8=緑／≤1.0=黄／>1.0 NG=赤 の順に横並びで描き、末尾に未検定の注記
    const SWATCH: f32 = 12.0;
    let mut x = x0;
    for (color, label) in [
        (theme::GOOD_GREEN, "≤0.8"),
        (theme::BEST_YELLOW, "≤1.0"),
        (theme::PARETO_RED, ">1.0 NG"),
    ] {
        painter.rect_filled(
            egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(SWATCH, SWATCH)),
            0.0,
            color,
        );
        let text_rect = painter.text(
            egui::pos2(x + SWATCH + 4.0, y),
            egui::Align2::LEFT_TOP,
            label,
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        x = text_rect.max.x + 12.0;
    }
    let untested_rect = painter.text(
        egui::pos2(x, y),
        egui::Align2::LEFT_TOP,
        "未検定: グレー",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
    y = untested_rect.max.y.max(y + SWATCH) + 4.0;

    if markers {
        let marker_rect = painter.text(
            egui::pos2(x0, y),
            egui::Align2::LEFT_TOP,
            "■ 検定位置（NG は数値付き）",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        y = marker_rect.max.y + 4.0;
    }

    if app.staleness.design_stale {
        painter.text(
            egui::pos2(x0, y),
            egui::Align2::LEFT_TOP,
            "⚠ モデルが編集されています。解析を再実行してください。",
            egui::FontId::proportional(12.0),
            theme::BEST_YELLOW,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cr(ratio: f64, ok: bool, components: Vec<CheckComponent>) -> CheckResult {
        CheckResult {
            ratio,
            ok,
            basis: "テスト規準".to_string(),
            detail: String::new(),
            components,
        }
    }

    // ── max_ratio_by_elem / max_ratio_by_node ──────────────────────────

    /// 同一部材に複数の検定位置がある場合、最大の検定比が採用される。
    #[test]
    fn max_ratio_by_elem_picks_max_ratio() {
        let id = ElemId(0);
        let map = max_ratio_by_elem([
            (id, Some((0.5, true))),
            (id, Some((0.9, true))),
            (id, Some((0.3, true))),
        ]);
        assert_eq!(map[&id].0, 0.9);
    }

    /// 1つでも NG（ok=false）の位置があれば、部材全体として NG（false）になる。
    #[test]
    fn max_ratio_by_elem_ng_propagates() {
        let id = ElemId(1);
        let map = max_ratio_by_elem([
            (id, Some((0.5, true))),
            (id, Some((1.2, false))),
            (id, Some((0.3, true))),
        ]);
        assert_eq!(map[&id].0, 1.2);
        assert!(!map[&id].1);
    }

    /// 全位置が OK なら OK フラグは true のまま。
    #[test]
    fn max_ratio_by_elem_all_ok_stays_ok() {
        let id = ElemId(2);
        let map = max_ratio_by_elem([(id, Some((0.4, true))), (id, Some((0.6, true)))]);
        assert!(map[&id].1);
    }

    /// 複数部材のデータは部材ごとに分離して集計される。
    #[test]
    fn max_ratio_by_elem_separates_by_id() {
        let a = ElemId(0);
        let b = ElemId(1);
        let map = max_ratio_by_elem([
            (a, Some((0.5, true))),
            (b, Some((1.5, false))),
            (a, Some((0.8, true))),
        ]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.8);
        assert!(map[&a].1);
        assert_eq!(map[&b].0, 1.5);
        assert!(!map[&b].1);
    }

    /// 空入力は空の集計結果を返す。
    #[test]
    fn max_ratio_by_elem_empty_input() {
        let map = max_ratio_by_elem(std::iter::empty::<(ElemId, Option<(f64, bool)>)>());
        assert!(map.is_empty());
    }

    /// フィルタ対象外（None）の位置は集計から除外される。全位置が None なら
    /// 部材自体が集計結果に含まれない（＝未検定として扱われ着色されない）。
    #[test]
    fn max_ratio_by_elem_none_is_excluded() {
        let id = ElemId(3);
        let map = max_ratio_by_elem([(id, None), (id, Some((0.6, true))), (id, None)]);
        assert_eq!(map.len(), 1);
        assert_eq!(map[&id].0, 0.6);

        let id2 = ElemId(4);
        let map2 = max_ratio_by_elem([(id2, None), (id2, None)]);
        assert!(!map2.contains_key(&id2));
    }

    /// 節点単位の集計も同じ規則（最大値採用・NG 伝播）で動作する。
    #[test]
    fn max_ratio_by_node_picks_max_and_propagates_ng() {
        let n = NodeId(0);
        let map = max_ratio_by_node([(n, Some((0.7, true))), (n, Some((1.1, false)))]);
        assert_eq!(map[&n].0, 1.1);
        assert!(!map[&n].1);
    }

    /// 節点集計は複数節点を分離して保持する。
    #[test]
    fn max_ratio_by_node_separates_by_id() {
        let a = NodeId(0);
        let b = NodeId(1);
        let map = max_ratio_by_node([(a, Some((0.2, true))), (b, Some((0.95, true)))]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.2);
        assert_eq!(map[&b].0, 0.95);
    }

    /// 節点集計の空入力は空の結果を返す。
    #[test]
    fn max_ratio_by_node_empty_input() {
        let map = max_ratio_by_node(std::iter::empty::<(NodeId, Option<(f64, bool)>)>());
        assert!(map.is_empty());
    }

    // ── ratio_for_filter ────────────────────────────────────────────────

    /// フィルタ=最大は cr.ratio / cr.ok をそのまま返す。
    #[test]
    fn ratio_for_filter_max_returns_ratio_and_ok() {
        let c = cr(1.13, false, vec![]);
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Max),
            Some((1.13, false))
        );
    }

    /// フィルタ=特定式は該当式の最大検定比を返し、OK 判定は 1.0 以下かで決まる
    /// （cr.ok とは独立、components の値のみで判定する）。
    #[test]
    fn ratio_for_filter_kind_picks_matching_component() {
        let c = cr(
            1.13,
            false,
            vec![
                CheckComponent {
                    kind: CheckKind::Bending,
                    ratio: 0.82,
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 1.13,
                },
            ],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Bending)),
            Some((0.82, true))
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Shear)),
            Some((1.13, false))
        );
    }

    /// 該当する式が components に無ければ None（フィルタ対象外）。
    #[test]
    fn ratio_for_filter_kind_absent_returns_none() {
        let c = cr(
            0.5,
            true,
            vec![CheckComponent {
                kind: CheckKind::Bending,
                ratio: 0.5,
            }],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Axial)),
            None
        );
    }

    /// 同一 kind の component が複数ある場合は最大値を採用する。
    #[test]
    fn ratio_for_filter_kind_multiple_same_kind_picks_max() {
        let c = cr(
            0.9,
            true,
            vec![
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 0.4,
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 0.9,
                },
            ],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Shear)),
            Some((0.9, true))
        );
    }

    // ── dominant_kind ───────────────────────────────────────────────────

    /// 最大検定比を与える component の kind を返す。
    #[test]
    fn dominant_kind_picks_max_component() {
        let c = cr(
            1.13,
            false,
            vec![
                CheckComponent {
                    kind: CheckKind::Bending,
                    ratio: 0.82,
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 1.13,
                },
            ],
        );
        assert_eq!(dominant_kind(&c), Some(CheckKind::Shear));
    }

    /// components が空なら None。
    #[test]
    fn dominant_kind_empty_components_returns_none() {
        let c = cr(0.5, true, vec![]);
        assert_eq!(dominant_kind(&c), None);
    }

    // ── available_check_kinds ───────────────────────────────────────────

    /// 出現した kind のみを CheckKind の定義順で返す。
    #[test]
    fn available_check_kinds_returns_present_kinds_in_definition_order() {
        let comps: Vec<Vec<CheckComponent>> = vec![
            vec![CheckComponent {
                kind: CheckKind::Shear,
                ratio: 0.5,
            }],
            vec![CheckComponent {
                kind: CheckKind::Bending,
                ratio: 0.6,
            }],
        ];
        let kinds = available_check_kinds(comps.iter().map(|c| c.as_slice()));
        // 定義順は Bending が Shear より先。
        assert_eq!(kinds, vec![CheckKind::Bending, CheckKind::Shear]);
    }

    /// 無関係な式（例: 軸力のみのモデルに存在しない「たわみ」）は含まれない。
    #[test]
    fn available_check_kinds_excludes_absent_kinds() {
        let comps: Vec<Vec<CheckComponent>> = vec![vec![CheckComponent {
            kind: CheckKind::Axial,
            ratio: 0.3,
        }]];
        let kinds = available_check_kinds(comps.iter().map(|c| c.as_slice()));
        assert_eq!(kinds, vec![CheckKind::Axial]);
    }

    /// 空入力は空の結果を返す。
    #[test]
    fn available_check_kinds_empty_input() {
        let kinds = available_check_kinds(std::iter::empty::<&[CheckComponent]>());
        assert!(kinds.is_empty());
    }

    // ── mid_label_text ──────────────────────────────────────────────────

    /// 支配式が分かる場合は数値と式名を併記する。
    #[test]
    fn mid_label_text_with_dominant() {
        assert_eq!(mid_label_text(1.13, Some(CheckKind::Shear)), "1.13 せん断");
    }

    /// 支配式が無い場合（フィルタ=特定式、または内訳なし）は数値のみ。
    #[test]
    fn mid_label_text_without_dominant() {
        assert_eq!(mid_label_text(0.82, None), "0.82");
    }

    // ── elem_check_positions / build_tooltip_rows ───────────────────────

    /// 指定した部材の検定位置のみを xi 順そのままに抽出する。
    #[test]
    fn elem_check_positions_filters_by_elem_id() {
        let a = ElemId(0);
        let b = ElemId(1);
        let checks = vec![
            (a, 0.0, cr(0.5, true, vec![])),
            (b, 0.0, cr(1.5, false, vec![])),
            (a, 1.0, cr(0.9, true, vec![])),
        ];
        let positions = elem_check_positions(&checks, a);
        assert_eq!(positions.len(), 2);
        assert_eq!(positions[0].0, 0.0);
        assert_eq!(positions[1].0, 1.0);
    }

    /// 位置×式の表データが、出現した式を列に、位置ごとの値・判定を行に持つ。
    #[test]
    fn build_tooltip_rows_builds_table() {
        let positions = vec![
            (
                0.0,
                true,
                vec![
                    CheckComponent {
                        kind: CheckKind::Bending,
                        ratio: 0.5,
                    },
                    CheckComponent {
                        kind: CheckKind::Shear,
                        ratio: 0.4,
                    },
                ],
            ),
            (
                0.5,
                false,
                vec![CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 1.13,
                }],
            ),
        ];
        let (kinds, rows) = build_tooltip_rows(&positions);
        assert_eq!(kinds, vec![CheckKind::Bending, CheckKind::Shear]);
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].values, vec![Some(0.5), Some(0.4)]);
        assert!(rows[0].ok);
        // 2 行目は Bending 式が無いため None。
        assert_eq!(rows[1].values, vec![None, Some(1.13)]);
        assert!(!rows[1].ok);
    }

    /// 検定位置が無ければ表も空。
    #[test]
    fn build_tooltip_rows_empty_positions() {
        let (kinds, rows) = build_tooltip_rows(&[]);
        assert!(kinds.is_empty());
        assert!(rows.is_empty());
    }
}
