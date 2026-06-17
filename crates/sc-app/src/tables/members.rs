use crate::app::App;

pub fn members_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};
    let n = app.model.elements.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "種別", "節点", "断面"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();
                let elem = &app.model.elements[i];
                row.col(|ui| {
                    ui.label(elem.id.0.to_string());
                });
                row.col(|ui| {
                    ui.label(format!("{:?}", elem.kind));
                });
                row.col(|ui| {
                    let ids: Vec<String> = elem.nodes.iter().map(|n| n.0.to_string()).collect();
                    ui.label(ids.join(","));
                });
                row.col(|ui| {
                    if let Some(sid) = elem.section {
                        ui.label(sid.0.to_string());
                    } else {
                        ui.label("―");
                    }
                });
            });
        });
}
