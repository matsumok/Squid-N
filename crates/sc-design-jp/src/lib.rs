pub mod allowable_stress;

#[cfg(feature = "p7")]
pub mod capacity_spectrum;
#[cfg(feature = "p7")]
pub mod holding_capacity;

pub use allowable_stress::*;

use sc_core::model::{Material, Section};

pub struct MemberForcesAt {
    pub pos: f64,
    pub n: f64,
    pub q: f64,
    pub m: f64,
}

pub struct CheckResult {
    pub ratio: f64,
    pub ok: bool,
    pub basis: String,
    pub detail: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadTerm {
    Long,
    Short,
}

pub struct DesignCtx {
    pub term: LoadTerm,
}

pub trait DesignCheck {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult;
}
