use crate::app::App;
use sc_core::ids::SectionId;

/// ビューアの表示モード。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum ViewMode {
    /// 形状のみ
    #[default]
    Shape,
    /// 変形図（線形静的結果）
    Deformed,
    /// モード形（固有値結果）
    Mode,
    /// N 図
    N,
    /// Q 図
    Q,
    /// M 図
    M,
    /// CMQ 図（両端固定端モーメント C とせん断 Q）
    Cmq,
}

/// 3D→2D 投影（簡易: 等角投影 + スケール + パン）。
#[derive(Clone)]
pub struct CameraState {
    scale: f32,
    pan: [f32; 2],
    yaw: f32,
    pitch: f32,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            scale: 1.0,
            pan: [0.0, 0.0],
            yaw: 30.0_f32.to_radians(),
            pitch: -20.0_f32.to_radians(),
        }
    }
}

fn project(p: [f64; 3], cam: &CameraState, center: [f32; 2]) -> [f32; 2] {
    let cy = cam.yaw.cos();
    let sy = cam.yaw.sin();
    let cp = cam.pitch.cos();
    let sp = cam.pitch.sin();
    // Yaw 回転 → Pitch 回転
    let x1 = p[0] as f32 * cy + p[1] as f32 * sy;
    let y1 = -p[0] as f32 * sy + p[1] as f32 * cy;
    let z1 = p[2] as f32;
    let y2 = y1 * cp - z1 * sp;
    let _z2 = y1 * sp + z1 * cp;
    [
        center[0] + cam.pan[0] + x1 * cam.scale,
        center[1] + cam.pan[1] - y2 * cam.scale,
    ]
}

pub fn viewer_panel(ui: &mut egui::Ui, app: &mut App) {
    let mut mode = app.view_mode;
    let mut deform_scale = app.deform_scale;
    let mut mode_idx = app.view_mode_idx;

    // --- コントロール ---
    ui.horizontal(|ui| {
        ui.label("表示:");
        ui.selectable_value(&mut mode, ViewMode::Shape, "形状");
        ui.selectable_value(&mut mode, ViewMode::Deformed, "変形");
        ui.selectable_value(&mut mode, ViewMode::Mode, "モード");
        ui.selectable_value(&mut mode, ViewMode::N, "N図");
        ui.selectable_value(&mut mode, ViewMode::Q, "Q図");
        ui.selectable_value(&mut mode, ViewMode::M, "M図");
        ui.selectable_value(&mut mode, ViewMode::Cmq, "CMQ図");
    });
    if matches!(mode, ViewMode::Deformed | ViewMode::Mode) {
        ui.horizontal(|ui| {
            ui.label("倍率:");
            ui.add(egui::Slider::new(&mut deform_scale, 1.0..=10000.0).logarithmic(true));
        });
    }
    if mode == ViewMode::Mode {
        let n_modes = app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .map(|m| m.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .results
                    .as_ref()
                    .and_then(|r| r.modal.as_ref())
                    .and_then(|m| m.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }

    app.view_mode = mode;
    app.deform_scale = deform_scale;
    app.view_mode_idx = mode_idx;

    // --- 梁作成モード ---
    // ON 中はクリックで節点を選び、2 点目で梁を生成する（OFF 中は部材クリック=断面割当）。
    ui.horizontal(|ui| {
        ui.toggle_value(&mut app.beam_draw_mode, "梁作成モード");
        if app.beam_draw_mode {
            match app.beam_draw_first {
                None => {
                    ui.label("始点の節点をクリック");
                }
                Some(nid) => {
                    ui.label(format!("始点 N{} 選択中 → 終点の節点をクリック", nid.0));
                    if ui.button("キャンセル").clicked() {
                        app.beam_draw_first = None;
                    }
                }
            }
        }
    });
    // モード OFF 時は始点選択をクリア
    if !app.beam_draw_mode {
        app.beam_draw_first = None;
    }

    // --- 断面割当 UI ---
    // focus_member を先にコピーして、後段の可変借用と競合しないようにする
    let focus_id: Option<sc_core::ids::ElemId> = app.nav.focus_member;
    // 存在確認もここで行い、ローカルに有効性と現在断面を取得
    let elem_info: Option<(sc_core::ids::ElemId, Option<SectionId>)> = focus_id.and_then(|eid| {
        app.model
            .elements
            .iter()
            .find(|e| e.id == eid)
            .map(|e| (e.id, e.section))
    });

    let mut pending_assign: Option<Option<SectionId>> = None;

    if let Some((elem_id, current_section)) = elem_info {
        ui.horizontal(|ui| {
            ui.label(format!("選択中の梁 #{}", elem_id.0));
            ui.label("断面:");
            let selected_text = current_section
                .map(|sid| format!("S{}", sid.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("viewer_assign_section")
                .selected_text(selected_text)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(current_section.is_none(), "―")
                        .clicked()
                    {
                        pending_assign = Some(None);
                    }
                    for sec in &app.model.sections {
                        if ui
                            .selectable_label(
                                current_section == Some(sec.id),
                                format!("S{}", sec.id.0),
                            )
                            .clicked()
                        {
                            pending_assign = Some(Some(sec.id));
                        }
                    }
                });
        });
        // クロージャ外で発行（借用ルール）
        if let Some(section) = pending_assign {
            app.undo.run(
                &mut app.model,
                Box::new(sc_edit::SetElementSection {
                    elem: elem_id,
                    section,
                }),
            );
            app.staleness.mark_edited();
        }
    } else {
        ui.label("ビューアで梁をクリックすると選択できます");
    }

    ui.separator();

    // --- 描画領域 ---
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height()),
        egui::Sense::click_and_drag(),
    );

    // カメラ操作（ドラッグ=パン/回転、ズーム）
    let mut cam = app.camera.clone();
    if response.dragged_by(egui::PointerButton::Primary) {
        let d = response.drag_delta();
        cam.pan[0] += d.x;
        cam.pan[1] += d.y;
    }
    if response.dragged_by(egui::PointerButton::Secondary) {
        let d = response.drag_delta();
        cam.yaw += d.x * 0.01;
        cam.pitch += d.y * 0.01;
        cam.pitch = cam.pitch.clamp(-1.4, 1.4);
    }
    let zoom_factor = ui.input(|i| i.zoom_delta());
    if zoom_factor != 1.0 {
        cam.scale *= zoom_factor;
    }
    cam.scale = cam.scale.clamp(0.001, 100000.0);

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_gray(30));

    let center = [rect.center().x, rect.center().y];

    // モデルが空なら何も描かない
    if app.model.nodes.is_empty() {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "モデルが空です",
            egui::FontId::default(),
            egui::Color32::from_gray(120),
        );
        return;
    }

    // 節点座標（変形・モード時は変位を加味）
    let disp = match mode {
        ViewMode::Deformed => app
            .results
            .as_ref()
            .and_then(|r| r.statics.first())
            .map(|(_, s)| s.disp.clone()),
        ViewMode::Mode => app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.shapes.get(mode_idx))
            .map(|shape| {
                // shape は自由度順の平坦ベクトル → [node][6] に分解
                let n = app.model.nodes.len();
                let mut disp = vec![[0.0; 6]; n];
                for (ni, row) in disp.iter_mut().enumerate() {
                    for (d, slot) in row.iter_mut().enumerate().take(6) {
                        let g = ni * 6 + d;
                        if g < shape.len() {
                            *slot = shape[g];
                        }
                    }
                }
                disp
            }),
        _ => None,
    };

    // 変形スケール: 最大変位をモデルサイズの 10% に正規化
    let deform_scale_actual = if disp.is_some() {
        let model_size = model_bbox_size(&app.model);
        let max_disp = disp
            .as_ref()
            .map(|d| {
                d.iter()
                    .map(|v| v[0].abs().max(v[1].abs()).max(v[2].abs()))
                    .fold(0.0_f64, f64::max)
            })
            .unwrap_or(0.0);
        if max_disp > 1e-12 {
            (model_size * 0.1 / max_disp) * deform_scale as f64
        } else {
            0.0
        }
    } else {
        0.0
    };

    let pts: Vec<[f32; 2]> = app
        .model
        .nodes
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let mut p = node.coord;
            if let Some(d) = &disp {
                p[0] += d[i][0] * deform_scale_actual;
                p[1] += d[i][1] * deform_scale_actual;
                p[2] += d[i][2] * deform_scale_actual;
            }
            project(p, &cam, center)
        })
        .collect();

    // --- クリック処理 ---
    if response.clicked() {
        if let Some(click_pos) = response.interact_pointer_pos() {
            if app.beam_draw_mode {
                // 梁作成モード：クリック位置に最も近い節点を選ぶ
                let mut best: Option<(usize, f32)> = None;
                for (i, &p) in pts.iter().enumerate() {
                    let d = (click_pos - egui::pos2(p[0], p[1])).length();
                    if best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        match app.beam_draw_first {
                            None => {
                                // 1 点目：始点として記憶
                                app.beam_draw_first = Some(node_id);
                            }
                            Some(first) => {
                                // 2 点目：始点と異なれば梁を生成。同一節点は無視。
                                if first != node_id {
                                    let new_id =
                                        sc_core::ids::ElemId(app.model.elements.len() as u32);
                                    let elem = sc_core::model::ElementData {
                                        id: new_id,
                                        kind: sc_core::model::ElementKind::Beam,
                                        nodes: [first, node_id].into_iter().collect(),
                                        section: None,
                                        material: None,
                                        local_axis: sc_core::model::LocalAxis {
                                            ref_vector: [0.0, 0.0, 1.0],
                                        },
                                        end_cond: [
                                            sc_core::model::EndCondition::Fixed,
                                            sc_core::model::EndCondition::Fixed,
                                        ],
                                        force_regime: sc_core::model::ForceRegime::Auto,
                                        rigid_zone: Default::default(),
                                    };
                                    app.undo
                                        .run(&mut app.model, Box::new(sc_edit::AddMember { elem }));
                                    app.staleness.mark_edited();
                                    app.nav.focus_member = Some(new_id);
                                }
                                // 次の梁に備えて始点をリセット
                                app.beam_draw_first = None;
                            }
                        }
                    }
                }
            } else {
                // 通常モード：クリック位置に最も近い部材線分を選び、閾値内なら選択。
                let mut best: Option<(sc_core::ids::ElemId, f32)> = None;
                for elem in &app.model.elements {
                    if elem.nodes.len() < 2 {
                        continue;
                    }
                    let n0 = elem.nodes[0].index();
                    let n1 = elem.nodes[1].index();
                    if n0 >= pts.len() || n1 >= pts.len() {
                        continue;
                    }
                    let a = egui::pos2(pts[n0][0], pts[n0][1]);
                    let b = egui::pos2(pts[n1][0], pts[n1][1]);
                    let d = dist_point_to_segment(click_pos, a, b);
                    if best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((elem.id, d));
                    }
                }
                // ピッキング許容距離（px）
                const PICK_THRESHOLD: f32 = 8.0;
                match best {
                    Some((id, d)) if d <= PICK_THRESHOLD => {
                        app.selection.members = vec![id];
                        app.nav.focus_member = Some(id);
                    }
                    _ => {
                        app.selection.members.clear();
                    }
                }
            }
        }
    }

    // 節点（梁作成モードの始点は強調表示）
    for (i, &p) in pts.iter().enumerate() {
        let is_first = app.beam_draw_first == Some(app.model.nodes[i].id);
        let (radius, color) = if is_first {
            (5.0, egui::Color32::from_rgb(255, 120, 120))
        } else {
            (3.0, egui::Color32::from_rgb(100, 200, 255))
        };
        painter.circle_filled(egui::pos2(p[0], p[1]), radius, color);
    }

    // 部材（線）
    let line_color = if matches!(mode, ViewMode::Deformed | ViewMode::Mode) {
        egui::Color32::from_rgb(255, 200, 80)
    } else {
        egui::Color32::from_gray(200)
    };
    for elem in &app.model.elements {
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 < pts.len() && n1 < pts.len() {
            painter.line_segment(
                [
                    egui::pos2(pts[n0][0], pts[n0][1]),
                    egui::pos2(pts[n1][0], pts[n1][1]),
                ],
                egui::Stroke::new(2.0, line_color),
            );
        }
    }

    // --- 応力図（N/Q/M）: 部材ローカルに沿って描画 ---
    if matches!(mode, ViewMode::N | ViewMode::Q | ViewMode::M) {
        draw_force_diagram(ui, &painter, app, mode, &pts, &cam, center);
    }
    if mode == ViewMode::Cmq {
        draw_cmq_diagram(&painter, app, &pts);
    }

    // 選択ハイライト
    for &elem_id in &app.selection.members {
        if let Some(elem) = app.model.elements.iter().find(|e| e.id == elem_id) {
            if elem.nodes.len() >= 2 {
                let n0 = elem.nodes[0].index();
                let n1 = elem.nodes[1].index();
                if n0 < pts.len() && n1 < pts.len() {
                    painter.line_segment(
                        [
                            egui::pos2(pts[n0][0], pts[n0][1]),
                            egui::pos2(pts[n1][0], pts[n1][1]),
                        ],
                        egui::Stroke::new(4.0, egui::Color32::from_rgb(255, 100, 100)),
                    );
                }
            }
        }
    }

    // カメラ状態を保存
    app.camera = cam;
}

/// 部材ローカルに沿って N/Q/M 図を描く。
fn draw_force_diagram(
    _ui: &mut egui::Ui,
    painter: &egui::Painter,
    app: &App,
    mode: ViewMode,
    pts: &[[f32; 2]],
    _cam: &CameraState,
    _center: [f32; 2],
) {
    let force_idx = match mode {
        ViewMode::N => 0, // N
        ViewMode::Q => 1, // Qy
        ViewMode::M => 5, // Mz
        _ => return,
    };
    let label = match mode {
        ViewMode::N => "N",
        ViewMode::Q => "Q",
        ViewMode::M => "M",
        _ => "",
    };

    let Some(results) = &app.results else {
        return;
    };
    let max_abs = results
        .member_forces
        .iter()
        .flat_map(|(_, mf)| mf.at.iter().map(|(_, f)| f[force_idx].abs()))
        .fold(0.0_f64, f64::max);
    if max_abs < 1e-12 {
        return;
    }
    let diagram_scale = 60.0 / max_abs as f32; // 最大値で60px

    for (elem_id, mf) in &results.member_forces {
        let elem = app.model.elements.iter().find(|e| e.id == *elem_id);
        let Some(elem) = elem else { continue };
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
        let dir = [p1.x - p0.x, p1.y - p0.y];
        let len = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
        if len < 1e-3 {
            continue;
        }
        let unit = [dir[0] / len, dir[1] / len];
        let normal = [-unit[1], unit[0]]; // 左90度

        // 評価位置の内力をプロット
        let mut diagram_pts: Vec<egui::Pos2> = Vec::new();
        for (xi, forces) in &mf.at {
            let val = forces[force_idx];
            let x = p0.x + dir[0] * *xi as f32;
            let y = p0.y + dir[1] * *xi as f32;
            let offset = val as f32 * diagram_scale;
            diagram_pts.push(egui::pos2(x + normal[0] * offset, y + normal[1] * offset));
        }
        if diagram_pts.len() >= 2 {
            // 図形（折れ線→ポリゴン）
            let mut poly = Vec::with_capacity(diagram_pts.len() + 2);
            poly.push(p0);
            poly.extend(diagram_pts);
            poly.push(p1);
            painter.add(egui::Shape::convex_polygon(
                poly,
                egui::Color32::from_rgba_unmultiplied(100, 200, 100, 60),
                egui::Stroke::new(1.5, egui::Color32::from_rgb(100, 220, 100)),
            ));
        }
    }

    // 凡例
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        format!("{}図 (max={:.2})", label, max_abs),
        egui::FontId::proportional(14.0),
        egui::Color32::from_gray(220),
    );
}

/// 部材ローカルに沿って CMQ 図（両端の固定端モーメント C とせん断 Q）を描く。
fn draw_cmq_diagram(painter: &egui::Painter, app: &App, pts: &[[f32; 2]]) {
    if app.beam_loads.is_empty() {
        painter.text(
            egui::pos2(
                painter.clip_rect().min.x + 10.0,
                painter.clip_rect().min.y + 30.0,
            ),
            egui::Align2::LEFT_TOP,
            "CMQ データがありません（床荷重分配を実行してください）",
            egui::FontId::proportional(13.0),
            egui::Color32::from_gray(160),
        );
        return;
    }

    let max_c = app
        .beam_loads
        .iter()
        .map(|bl| bl.cmq.c_i.abs().max(bl.cmq.c_j.abs()))
        .fold(0.0_f64, f64::max);
    let max_q = app
        .beam_loads
        .iter()
        .map(|bl| bl.cmq.q_i.abs().max(bl.cmq.q_j.abs()))
        .fold(0.0_f64, f64::max);
    if max_c < 1e-12 && max_q < 1e-12 {
        return;
    }
    let c_scale = 60.0 / max_c.max(1e-12) as f32;
    let q_scale = 60.0 / max_q.max(1e-12) as f32;

    for bl in &app.beam_loads {
        let elem = app.model.elements.iter().find(|e| e.id == bl.elem);
        let Some(elem) = elem else { continue };
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
        let dir = [p1.x - p0.x, p1.y - p0.y];
        let len = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
        if len < 1e-3 {
            continue;
        }
        let unit = [dir[0] / len, dir[1] / len];
        let normal = [-unit[1], unit[0]];

        // C 図（モーメント）: 両端の c_i, c_j を結ぶ折れ線ポリゴン
        let c_poly = vec![
            p0,
            egui::pos2(
                p0.x + normal[0] * bl.cmq.c_i as f32 * c_scale,
                p0.y + normal[1] * bl.cmq.c_i as f32 * c_scale,
            ),
            egui::pos2(
                p1.x + normal[0] * bl.cmq.c_j as f32 * c_scale,
                p1.y + normal[1] * bl.cmq.c_j as f32 * c_scale,
            ),
            p1,
        ];
        painter.add(egui::Shape::convex_polygon(
            c_poly,
            egui::Color32::from_rgba_unmultiplied(80, 140, 255, 60),
            egui::Stroke::new(1.5, egui::Color32::from_rgb(100, 160, 255)),
        ));

        // Q 図（せん断）: 両端の q_i, q_j を結ぶ折れ線ポリゴン（反対側に描画）
        let q_poly = vec![
            p0,
            egui::pos2(
                p0.x - normal[0] * bl.cmq.q_i as f32 * q_scale,
                p0.y - normal[1] * bl.cmq.q_i as f32 * q_scale,
            ),
            egui::pos2(
                p1.x - normal[0] * bl.cmq.q_j as f32 * q_scale,
                p1.y - normal[1] * bl.cmq.q_j as f32 * q_scale,
            ),
            p1,
        ];
        painter.add(egui::Shape::convex_polygon(
            q_poly,
            egui::Color32::from_rgba_unmultiplied(255, 140, 80, 60),
            egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 160, 100)),
        ));
    }

    // 凡例
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        format!("CMQ図 C(max={:.2}) 青／Q(max={:.2}) 橙", max_c, max_q),
        egui::FontId::proportional(14.0),
        egui::Color32::from_gray(220),
    );
}

/// 点 p から線分 ab までの最短距離（スクリーン座標, px）。
fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len_sq = ab.x * ab.x + ab.y * ab.y;
    if len_sq < 1e-6 {
        return ap.length();
    }
    let t = ((ap.x * ab.x + ap.y * ab.y) / len_sq).clamp(0.0, 1.0);
    let proj = egui::pos2(a.x + ab.x * t, a.y + ab.y * t);
    (p - proj).length()
}

/// モデルのバウンディングボックス対角線長。
fn model_bbox_size(model: &sc_core::model::Model) -> f64 {
    if model.nodes.is_empty() {
        return 1.0;
    }
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for n in &model.nodes {
        for k in 0..3 {
            min[k] = min[k].min(n.coord[k]);
            max[k] = max[k].max(n.coord[k]);
        }
    }
    ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2) + (max[2] - min[2]).powi(2)).sqrt()
}
