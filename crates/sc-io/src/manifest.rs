#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Manifest {
    pub schema_version: u32,
    pub units: String,
    pub created_by: String,
    pub entries: Vec<EntryHash>,
    // 将来: results_inclusion: ResultInclusion (内包/外部, 50MB閾値 R27) ── P0 では未使用
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EntryHash {
    pub name: String,
    pub sha256: String,
}
