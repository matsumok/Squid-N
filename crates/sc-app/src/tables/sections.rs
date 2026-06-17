use crate::app::App;

pub fn sections_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};
    let n = app.model.sections.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(100.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "A", "Iy"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();
                let sec = &app.model.sections[i];
                row.col(|ui| {
                    ui.label(sec.id.0.to_string());
                });
                row.col(|ui| {
                    ui.label(&sec.name);
                });
                row.col(|ui| {
                    ui.label(format!("{:.1}", sec.area));
                });
                row.col(|ui| {
                    ui.label(format!("{:.1}", sec.iy));
                });
            });
        });
}
