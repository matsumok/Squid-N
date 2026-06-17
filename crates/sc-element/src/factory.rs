use crate::behavior::{ElemState, ElementBehavior};
use sc_core::model::{ElementData, ElementKind, Model};

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::PanelZone => (
            Box::new(crate::panel::PanelZone::new(data, model)),
            ElemState::default(),
        ),
        other => panic!("element kind {:?} not supported in P1", other),
    }
}
