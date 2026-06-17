/// チェックポイント／再開（設計書 §12.4）。
/// 長時間の非線形／時刻歴の再開・巻き戻しのため、解析状態をバイナリ保存する。

/// P5 §6 の StateSnapshot を直列化したバイト列（全 ElemState・全材料 committed）。
#[derive(serde::Serialize, serde::Deserialize)]
pub struct StateBlob {
    pub bytes: Vec<u8>,
}

/// チェックポイント内容（設計書 §12.4）。
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Checkpoint {
    pub schema_version: u32,
    pub model_hash: String,
    pub step: u64,
    pub time: f64,
    pub disp: Vec<f64>,
    pub vel: Vec<f64>,
    pub accel: Vec<f64>,
    pub state: StateBlob,
}

/// チェックポイントを高速バイナリ形式（bincode）で保存する。
pub fn save_checkpoint(dir: &std::path::Path, cp: &Checkpoint) -> std::io::Result<()> {
    let _ = (dir, cp);
    todo!("save_checkpoint")
}

/// チェックポイントを高速バイナリ形式から読み込む。
pub fn load_checkpoint(dir: &std::path::Path) -> std::io::Result<Checkpoint> {
    let _ = dir;
    todo!("load_checkpoint")
}
