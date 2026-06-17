use crate::app::App;

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};
    let n = app.model.nodes.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .columns(Column::initial(80.0), 3)
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "X", "Y", "Z", "拘束"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();
                let node = &app.model.nodes[i];
                row.col(|ui| {
                    ui.label(node.id.0.to_string());
                });
                for k in 0..3 {
                    row.col(|ui| {
                        ui.label(format!("{:.1}", node.coord[k]));
                    });
                }
                row.col(|ui| {
                    ui.label(format!("{:?}", node.restraint));
                });
            });
        });
}
