use arrow::record_batch::RecordBatch;
use sc_core::ids::{ElemId, NodeId};

pub type CaseId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResultKind {
    NodalDisp,
    MemberForce,
    Story,
    Modal,
}

pub struct ResultQuery {
    pub case: CaseId,
    pub kind: ResultKind,
    pub node_filter: Option<Vec<NodeId>>,
    pub member_filter: Option<Vec<ElemId>>,
    pub step_range: Option<(u64, u64)>,
}

pub struct ResultBatch {
    pub batch: RecordBatch,
}

pub trait ResultWriter {
    fn write_rows(&mut self, batch: &RecordBatch);
    fn finish(self: Box<Self>);
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultManifest {
    pub entries: Vec<ResultEntry>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultEntry {
    pub case: CaseId,
    pub kind: ResultKind,
    pub rows: u64,
    pub path: String,
}

pub trait ResultStore {
    fn writer(&mut self, case: CaseId, kind: ResultKind) -> Box<dyn ResultWriter>;
    fn query(&self, q: &ResultQuery) -> ResultBatch;
    fn manifest(&self) -> &ResultManifest;
}
