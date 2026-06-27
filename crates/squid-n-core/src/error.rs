use thiserror::Error;

#[derive(Error, Debug, Clone, PartialEq, Eq)]
pub enum CoreError {
    #[error("duplicate id: {0}")]
    DuplicateId(String),
    #[error("dangling reference: {0}")]
    DanglingRef(String),
    #[error("index mismatch: {0}")]
    IndexMismatch(String),
}
