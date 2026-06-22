//! チェックポイント／再開（設計書 §12.4）。
//! 長時間の非線形／時刻歴の再開・巻き戻しのため、解析状態をバイナリ保存する。

use sha2::{Digest, Sha256};
use std::io;
use std::path::Path;

/// P5 §6 の StateSnapshot を直列化したバイト列（全 ElemState・全材料 committed）。
/// 線形時刻歴では空配列でよい。
#[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
pub struct StateBlob {
    pub element_states: Vec<Vec<u8>>,
}

/// チェックポイント内容（設計書 §12.4）。
#[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
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

const CHECKPOINT_DIR: &str = "checkpoint";
const CHECKPOINT_FILE: &str = "checkpoint.bin";
const CHECKPOINT_TMP: &str = "checkpoint.tmp";

/// チェックポイントを高速バイナリ形式（bincode）で保存する。
/// 原子的書込のため、一時ファイルに書いてからリネームする。
pub fn save_checkpoint(dir: &Path, cp: &Checkpoint) -> io::Result<()> {
    let cp_dir = dir.join(CHECKPOINT_DIR);
    std::fs::create_dir_all(&cp_dir)?;

    let tmp_path = cp_dir.join(CHECKPOINT_TMP);
    let final_path = cp_dir.join(CHECKPOINT_FILE);

    let encoded = bincode::serialize(cp).map_err(io::Error::other)?;
    std::fs::write(&tmp_path, &encoded)?;
    std::fs::rename(&tmp_path, &final_path)?;

    Ok(())
}

/// チェックポイントを高速バイナリ形式から読み込む。
pub fn load_checkpoint(dir: &Path) -> io::Result<Checkpoint> {
    let path = dir.join(CHECKPOINT_DIR).join(CHECKPOINT_FILE);
    let bytes = std::fs::read(&path)?;
    bincode::deserialize(&bytes).map_err(io::Error::other)
}

/// チェックポイントの model_hash と期待値が一致するか検証する。
pub fn verify_model_hash(cp: &Checkpoint, expected_hash: &str) -> Result<(), String> {
    if cp.model_hash != expected_hash {
        return Err(format!(
            "model hash mismatch: checkpoint={} expected={}",
            cp.model_hash, expected_hash
        ));
    }
    Ok(())
}

/// モデルの安定なハッシュ（SHA-256）を計算する。
/// JSON 文字列化してからハッシュ化することで、フィールド順序が安定する。
pub fn compute_model_hash(model: &sc_core::model::Model) -> String {
    let json = serde_json::to_string(model).expect("Model serialization must not fail");
    let mut hasher = Sha256::new();
    hasher.update(json.as_bytes());
    format!("{:x}", hasher.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_checkpoint(model_hash: &str, step: u64) -> Checkpoint {
        Checkpoint {
            schema_version: 1,
            model_hash: model_hash.to_string(),
            step,
            time: step as f64 * 0.01,
            disp: vec![0.0, 0.0, 0.0],
            vel: vec![0.0, 0.0, 0.0],
            accel: vec![0.0, 0.0, 0.0],
            state: StateBlob {
                element_states: vec![],
            },
        }
    }

    #[test]
    fn test_checkpoint_save_load_roundtrip() {
        let dir = std::env::temp_dir().join("cp_roundtrip_test");
        let _ = std::fs::remove_dir_all(&dir);
        let cp = make_checkpoint("abc123", 42);

        save_checkpoint(&dir, &cp).unwrap();
        let loaded = load_checkpoint(&dir).unwrap();

        assert_eq!(loaded, cp);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_model_hash_mismatch_rejected() {
        let cp = make_checkpoint("abc123", 0);
        let result = verify_model_hash(&cp, "xyz789");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("model hash mismatch"));
    }

    #[test]
    fn test_model_hash_same_model_consistent() {
        let model1 = sc_core::model::Model::default();
        let model2 = sc_core::model::Model::default();
        let h1 = compute_model_hash(&model1);
        let h2 = compute_model_hash(&model2);
        assert_eq!(h1, h2);
    }

    #[test]
    fn test_load_nonexistent_returns_error() {
        let dir = std::env::temp_dir().join("cp_nonexistent_test");
        let _ = std::fs::remove_dir_all(&dir);
        // dir は存在するが checkpoint/ は存在しない → NotFound
        std::fs::create_dir_all(&dir).unwrap();
        let result = load_checkpoint(&dir);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::NotFound);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
