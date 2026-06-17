use crate::scz::{IoError, CURRENT_SCHEMA_VERSION};

pub fn migrate(version: u32, bytes: Vec<u8>) -> Result<Vec<u8>, IoError> {
    match version {
        v if v == CURRENT_SCHEMA_VERSION => Ok(bytes),
        v => Err(IoError::UnsupportedVersion(v)),
    }
}
