
use super::*;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{ElementData, ElementKind, LocalAxis, Node, Section};

fn sample_model() -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: squid_n_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(squid_n_core::ids::StoryId(0)),
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "H-400".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 2000.0,
            j: 50.0,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_query_model_nodes() {
    let m = sample_model();
    let items = query_model(&m, "node", None);
    assert_eq!(items.len(), 2);
    assert_eq!(items[0]["id"], 0);
    assert_eq!(items[1]["story"], 0);
}

#[test]
fn test_query_model_elements_and_sections() {
    let m = sample_model();
    assert_eq!(query_model(&m, "member", None).len(), 1);
    let secs = query_model(&m, "section", None);
    assert_eq!(secs.len(), 1);
    assert_eq!(secs[0]["name"], "H-400");
}

#[test]
fn test_query_model_filter() {
    let m = sample_model();
    // 名前で絞り込み（断面名 H-400 を含むものだけ）。
    assert_eq!(query_model(&m, "section", Some("H-400")).len(), 1);
    assert_eq!(query_model(&m, "section", Some("RC")).len(), 0);
}

#[test]
fn test_query_model_unknown_kind() {
    let m = sample_model();
    assert!(query_model(&m, "bogus", None).is_empty());
}

#[test]
fn test_job_registry_lifecycle() {
    let mut reg = JobRegistry::new();
    let id = reg.register(JobKind::LinearStatic);
    assert!(matches!(reg.get(&id).unwrap().status, JobStatus::Queued));
    reg.update(&id, JobStatus::Running { progress: 0.5 });
    assert!(matches!(
        reg.get(&id).unwrap().status,
        JobStatus::Running { progress } if (progress - 0.5).abs() < 1e-6
    ));
    reg.update(
        &id,
        JobStatus::Done {
            result_ref: "r1".into(),
        },
    );
    assert!(matches!(
        &reg.get(&id).unwrap().status,
        JobStatus::Done { result_ref } if result_ref == "r1"
    ));
    // 異なる ID は別ジョブ。
    let id2 = reg.register(JobKind::Eigen);
    assert_ne!(id, id2);
    assert!(reg.get("nonexistent").is_none());
}
