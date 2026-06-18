use sc_edit::UndoStack;

pub enum Tab {
    Nodes,
    Members,
    Sections,
    Loads,
    Viewer,
    Design,
}

#[derive(Default)]
pub struct Selection {
    pub nodes: Vec<sc_core::ids::NodeId>,
    pub members: Vec<sc_core::ids::ElemId>,
}

pub struct ResultsBundle {
    pub statics: Vec<(sc_core::ids::LoadCaseId, sc_solver::linear::StaticOnce)>,
    pub modal: Option<sc_solver::eigen::ModalResult>,
    pub member_forces: Vec<(sc_core::ids::ElemId, sc_element::beam::MemberForces)>,
    pub checks: Vec<(sc_core::ids::ElemId, f64, sc_design_jp::CheckResult)>,
}

pub struct App {
    pub model: sc_core::model::Model,
    pub results: Option<ResultsBundle>,
    pub selection: Selection,
    pub undo: UndoStack,
    pub active_tab: Tab,
}

impl Default for App {
    fn default() -> Self {
        Self {
            model: sc_core::model::Model::default(),
            results: None,
            selection: Selection::default(),
            undo: UndoStack::new(),
            active_tab: Tab::Nodes,
        }
    }
}

#[cfg(feature = "gui")]
impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        egui::TopBottomPanel::top("tabs").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let tabs = [
                    ("節点", Tab::Nodes),
                    ("部材", Tab::Members),
                    ("断面", Tab::Sections),
                    ("荷重", Tab::Loads),
                    ("3D", Tab::Viewer),
                    ("設計", Tab::Design),
                ];
                for (label, tab) in &tabs {
                    let selected =
                        std::mem::discriminant(&self.active_tab) == std::mem::discriminant(tab);
                    if ui.selectable_label(selected, *label).clicked() {
                        self.active_tab = match tab {
                            Tab::Nodes => Tab::Nodes,
                            Tab::Members => Tab::Members,
                            Tab::Sections => Tab::Sections,
                            Tab::Loads => Tab::Loads,
                            Tab::Viewer => Tab::Viewer,
                            Tab::Design => Tab::Design,
                        };
                    }
                }
            });
        });
        egui::CentralPanel::default().show(ctx, |ui| match self.active_tab {
            Tab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            Tab::Members => crate::tables::members::members_table(ui, self),
            Tab::Sections => crate::tables::sections::sections_table(ui, self),
            Tab::Loads => crate::tables::loads::loads_table(ui, self),
            Tab::Viewer => crate::viewer::viewer_panel(ui, self),
            Tab::Design => crate::design_view::design_table(ui, self),
        });
    }
}
