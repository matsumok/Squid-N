use crate::app::App;

pub fn loads_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};
    let n = app.model.load_cases.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(100.0))
        .column(Column::initial(80.0))
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "荷重数"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();
                let lc = &app.model.load_cases[i];
                row.col(|ui| {
                    ui.label(lc.id.0.to_string());
                });
                row.col(|ui| {
                    ui.label(&lc.name);
                });
                row.col(|ui| {
                    ui.label(lc.nodal.len().to_string());
                });
            });
        });
}
