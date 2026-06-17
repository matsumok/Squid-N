use sc_core::model::Model;

#[derive(Debug, thiserror::Error)]
pub enum StbError {
    #[error("xml parse: {0}")]
    Parse(String),
    #[error("unsupported version: {0}")]
    Version(String),
    #[error("unmappable element: {0}")]
    Unmappable(String),
}

pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    let _ = xml;
    Err(StbError::Version(
        "ST-Bridge 2.0 import not yet implemented".into(),
    ))
}

pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    let _ = model;
    Err(StbError::Version(
        "ST-Bridge 2.0 export not yet implemented".into(),
    ))
}
