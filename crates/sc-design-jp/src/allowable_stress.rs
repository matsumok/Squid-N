use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt};
use sc_core::model::{Material, Section};

pub struct SteelDesign;

fn allowable_steel_stress(grade: &str, long_term: bool) -> f64 {
    let base = match grade {
        "SS400" | "SN400" => 235.0,
        "SS490" => 235.0,
        "SN490" => 325.0,
        _ => 235.0,
    };
    if long_term {
        base / 1.5
    } else {
        base
    }
}

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let grade = &mat.name;
        let long_term = ctx.term == LoadTerm::Long;
        let fb = allowable_steel_stress(grade, long_term);
        let fs = fb / 3.0_f64.sqrt();

        let z = if sec.iz != 0.0 { sec.iz } else { sec.iy };
        let z_eff = if z > 0.0 { z } else { 1.0 };
        let sigma_b = forces.m.abs() * 1000.0 / z_eff;

        let ratio_b = if fb > 0.0 { sigma_b / fb } else { 0.0 };
        let ratio_s = if fs > 0.0 { forces.q.abs() / fs } else { 0.0 };
        let ratio = ratio_b.max(ratio_s);

        let basis = if long_term {
            "令90条 長期許容応力度 F/1.5"
        } else {
            "令90条 短期許容応力度 F"
        };

        let detail = format!(
            "σ={:.4}, fb={:.4}, τ={:.4}, fs={:.4}",
            sigma_b, fb, forces.q, fs
        );

        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: basis.to_string(),
            detail,
        }
    }
}

pub struct RcDesign;

fn allowable_concrete_stress(fc: f64, long_term: bool) -> f64 {
    if long_term {
        fc / 3.0
    } else {
        fc / 1.5
    }
}

fn allowable_rebar_stress(grade: &str, long_term: bool) -> f64 {
    let base = match grade {
        "SD295" => 195.0,
        "SD345" => 215.0,
        _ => 195.0,
    };
    if long_term {
        base
    } else {
        (base * 1.5).min(295.0)
    }
}

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let long_term = ctx.term == LoadTerm::Long;
        let fc = mat.young;
        let _fc_allow = allowable_concrete_stress(fc, long_term);

        let z = if sec.iz != 0.0 { sec.iz } else { sec.iy };
        let z_eff = if z > 0.0 { z } else { 1.0 };
        let sigma_c = forces.m.abs() * 1000.0 / z_eff;

        let rebar_grade = "SD345";
        let ft = allowable_rebar_stress(rebar_grade, long_term);

        let ratio = if ft > 0.0 { sigma_c / ft } else { 0.0 };

        let basis = if long_term {
            "令90条・令91条 長期"
        } else {
            "令90条・令91条 短期"
        };

        let detail = format!("σc={:.4}, ft={:.4}", sigma_c, ft);

        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: basis.to_string(),
            detail,
        }
    }
}

pub fn combined_stress_ratio(
    axial_stress: f64,
    bending_stress: f64,
    allowable_axial: f64,
    allowable_bending: f64,
) -> f64 {
    let axial_ratio = if allowable_axial != 0.0 {
        axial_stress / allowable_axial
    } else {
        0.0
    };
    let bend_ratio = if allowable_bending != 0.0 {
        bending_stress / allowable_bending
    } else {
        0.0
    };
    axial_ratio + bend_ratio
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ss400_allowable_long() {
        let fa = allowable_steel_stress("SS400", true);
        assert!((fa - 235.0 / 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_ss400_allowable_short() {
        let fa = allowable_steel_stress("SS400", false);
        assert!((fa - 235.0).abs() < 1e-6);
    }

    #[test]
    fn test_steel_check_bending() {
        let sec = Section {
            id: sc_core::ids::SectionId(0),
            name: "H-200x100".into(),
            area: 1000.0,
            iy: 1e7,
            iz: 5e6,
            j: 1e5,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
            panel_thickness: None,
            thickness: None,
        };
        let mat = Material {
            id: sc_core::ids::MaterialId(0),
            name: "SN400".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
        };
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            q: 0.0,
            m: 100.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        let expected_ratio = (100.0 * 1000.0 / 5e6) / (235.0 / 1.5);
        assert!((result.ratio - expected_ratio).abs() < 1e-6);
        assert!(result.ok);
    }

    #[test]
    fn test_concrete_allowable_long() {
        let fa = allowable_concrete_stress(24.0, true);
        assert!((fa - 24.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_rebar_allowable_sd345() {
        let ft = allowable_rebar_stress("SD345", true);
        assert!((ft - 215.0).abs() < 1e-6);
    }

    #[test]
    fn test_rebar_allowable_short_upper_bound() {
        let ft = allowable_rebar_stress("SD345", false);
        assert!((ft - 295.0).abs() < 1e-6);
    }
}
