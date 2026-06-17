use sc_core::ids::{ElemId, StoryId};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemberRank {
    FA,
    FB,
    FC,
    FD,
}

pub struct StoryCheck {
    pub story: StoryId,
    pub rs: f64,
    pub re: f64,
    pub ds: f64,
    pub fes: f64,
    pub qu: f64,
    pub qud: f64,
    pub qun: f64,
    pub drift_angle: f64,
    pub ok: bool,
}

pub struct HoldingCapacityResult {
    pub stories: Vec<StoryCheck>,
    pub member_ranks: Vec<(ElemId, MemberRank)>,
}

// ===== T1: 剛性率 Rs・層間変形角 (§4) =====

pub fn check_story_drift(story_height: f64, interstory_drift: f64) -> bool {
    let angle = interstory_drift / story_height;
    angle <= 1.0 / 200.0
}

/// 全層の剛性率 Rs_i を計算する。
/// Ks_i = h_i / δ_i,  Rs_i = Ks_i / mean(Ks)
pub fn stiffness_ratios(story_heights: &[f64], story_drifts: &[f64]) -> Vec<f64> {
    let ks: Vec<f64> = story_heights
        .iter()
        .zip(story_drifts)
        .map(|(h, d)| if *d == 0.0 { 0.0 } else { h / d })
        .collect();
    let n = ks.len() as f64;
    if n == 0.0 {
        return vec![];
    }
    let mean = ks.iter().sum::<f64>() / n;
    if mean == 0.0 {
        return vec![1.0; ks.len()];
    }
    ks.iter().map(|k| k / mean).collect()
}

// ===== T2: 偏心率 Re (§5.2) =====

/// 偏心距離 e を弾力半径 r で割った偏心率。
pub fn eccentricity_ratio(e: f64, r: f64) -> f64 {
    if r == 0.0 {
        return 0.0;
    }
    (e / r).abs()
}

// ===== T3: Fs / Fe / Fes (§5.3) =====

/// 剛性率 Rs から Fs を算定（告示1792）。
/// Rs ≥ 0.6 → Fs = 1.0
/// Rs < 0.6 → Fs = 2.0 − Rs/0.6
pub fn fs(rs: f64) -> f64 {
    if rs >= 0.6 {
        1.0
    } else {
        2.0 - rs / 0.6
    }
}

/// 偏心率 Re から Fe を算定（告示1792）。
/// Re ≤ 0.15 → Fe = 1.0
/// Re > 0.15 → Fe = 1.0 + 0.5·(Re − 0.15)/0.15（最大 1.5）
pub fn fe(re: f64) -> f64 {
    if re <= 0.15 {
        1.0
    } else {
        (1.0 + 0.5 * (re - 0.15) / 0.15).min(1.5)
    }
}

/// 形状係数 Fes = Fs · Fe。
pub fn fes(rs: f64, re: f64) -> f64 {
    fs(rs) * fe(re)
}

// ===== T4: Ds 自動分類 (§7) =====

/// 架構種別。Ds 表の行を選ぶ。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameType {
    RcFrame,
    RcWall,
    SteelFrame,
    SteelBrace,
}

/// 告示1792 の Ds 値。
pub fn ds_value(frame: FrameType, rank: MemberRank) -> f64 {
    use FrameType::*;
    use MemberRank::*;
    match (frame, rank) {
        (RcFrame, FA) => 0.30,
        (RcFrame, FB) => 0.35,
        (RcFrame, FC) => 0.40,
        (RcFrame, FD) => 0.45,
        (RcWall, FA) => 0.35,
        (RcWall, FB) => 0.40,
        (RcWall, FC) => 0.45,
        (RcWall, FD) => 0.55,
        (SteelFrame, FA) => 0.25,
        (SteelFrame, FB) => 0.30,
        (SteelFrame, FC) => 0.35,
        (SteelFrame, FD) => 0.40,
        (SteelBrace, FA) => 0.30,
        (SteelBrace, FB) => 0.35,
        (SteelBrace, FC) => 0.40,
        (SteelBrace, FD) => 0.50,
    }
}

// ===== T6: Qun 比較・判定 (§3) =====

use sc_solver::pushover::PushoverResult;

/// 層ごとに Qun = Ds * Fes * Qud を計算し、Qu ≥ Qun を判定する。
pub fn check_holding_capacity(
    pushover: &PushoverResult,
    qu_by_story: &[f64],
    qud_by_story: &[f64],
    ds_by_story: &[f64],
    fes_by_story: &[f64],
    story_heights: &[f64],
) -> HoldingCapacityResult {
    let last_step = pushover.steps.last();
    let n = qu_by_story.len();

    let stories: Vec<StoryCheck> = (0..n)
        .map(|i| {
            let story = StoryId(i as u32);
            let drift = last_step
                .and_then(|s| s.story_drifts.get(i))
                .copied()
                .unwrap_or(0.0);
            let height = *story_heights.get(i).unwrap_or(&1.0);
            let drift_angle = if height == 0.0 { 0.0 } else { drift / height };
            let qu = qu_by_story[i];
            let qud = qud_by_story[i];
            let ds = ds_by_story[i];
            let f = fes_by_story[i];
            let qun = ds * f * qud;
            let ok = qu >= qun;
            StoryCheck {
                story,
                rs: 0.0,
                re: 0.0,
                ds,
                fes: f,
                qu,
                qud,
                qun,
                drift_angle,
                ok,
            }
        })
        .collect();

    HoldingCapacityResult {
        stories,
        member_ranks: vec![],
    }
}

// ===== T2: 剛心（D値法）stub (§5.1) =====
// ===== T4: 部材ランク・層Ds判定 stub (§7) =====
// ===== T5: パネルせん断検定 stub (§6) =====

#[cfg(test)]
mod tests {
    use super::*;

    // ---- T1 ----
    #[test]
    fn test_stiffness_ratios_example() {
        let heights = vec![1.0, 1.0, 1.0];
        let drifts = vec![1.0 / 200.0, 1.0 / 150.0, 1.0 / 250.0];
        let rs = stiffness_ratios(&heights, &drifts);
        assert_eq!(rs.len(), 3);
        for (r, expected) in rs.iter().zip([1.0, 0.75, 1.25].iter()) {
            assert!((r - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn test_stiffness_ratios_empty() {
        assert!(stiffness_ratios(&[], &[]).is_empty());
    }

    #[test]
    fn test_stiffness_ratios_zero_drift() {
        let rs = stiffness_ratios(&[1.0], &[0.0]);
        assert_eq!(rs, vec![1.0]);
    }

    #[test]
    fn test_check_story_drift_ok() {
        assert!(check_story_drift(3.0, 0.01));
        assert!(!check_story_drift(3.0, 0.02));
    }

    // ---- T2 ----
    #[test]
    fn test_eccentricity_ratio_basic() {
        assert!((eccentricity_ratio(1.5, 3.0) - 0.5).abs() < 1e-9);
        assert_eq!(eccentricity_ratio(1.5, 0.0), 0.0);
    }

    // ---- T3 ----
    #[test]
    fn test_fs_ge_06() {
        assert!((fs(0.6) - 1.0).abs() < 1e-9);
        assert!((fs(1.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fs_lt_06() {
        assert!((fs(0.3) - 1.5).abs() < 1e-9);
        assert!((fs(0.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_fe_le_015() {
        assert!((fe(0.15) - 1.0).abs() < 1e-9);
        assert!((fe(0.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fe_gt_015() {
        assert!((fe(0.30) - 1.5).abs() < 1e-9);
        assert!((fe(0.45) - 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_fes_default() {
        let f = fes(0.8, 0.1);
        assert!((f - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fes_example() {
        assert!((fes(0.3, 0.30) - 2.25).abs() < 1e-9);
    }

    // ---- T4 ----
    #[test]
    fn test_ds_value_rc_frame() {
        assert!((ds_value(FrameType::RcFrame, MemberRank::FA) - 0.30).abs() < 1e-9);
        assert!((ds_value(FrameType::RcFrame, MemberRank::FD) - 0.45).abs() < 1e-9);
    }

    #[test]
    fn test_ds_value_steel_frame() {
        assert!((ds_value(FrameType::SteelFrame, MemberRank::FA) - 0.25).abs() < 1e-9);
        assert!((ds_value(FrameType::SteelFrame, MemberRank::FD) - 0.40).abs() < 1e-9);
    }

    #[test]
    fn test_ds_value_all_combinations() {
        for (f, r, expected) in [
            (FrameType::RcFrame, MemberRank::FA, 0.30),
            (FrameType::RcFrame, MemberRank::FB, 0.35),
            (FrameType::RcFrame, MemberRank::FC, 0.40),
            (FrameType::RcFrame, MemberRank::FD, 0.45),
            (FrameType::RcWall, MemberRank::FA, 0.35),
            (FrameType::RcWall, MemberRank::FD, 0.55),
            (FrameType::SteelFrame, MemberRank::FA, 0.25),
            (FrameType::SteelFrame, MemberRank::FD, 0.40),
            (FrameType::SteelBrace, MemberRank::FA, 0.30),
            (FrameType::SteelBrace, MemberRank::FD, 0.50),
        ] {
            assert!(
                (ds_value(f, r) - expected).abs() < 1e-9,
                "ds_value({:?}, {:?}) should be {}, got {}",
                f,
                r,
                expected,
                ds_value(f, r)
            );
        }
    }

    // ---- T6 ----
    #[test]
    fn test_check_holding_capacity_basic() {
        let pushover = PushoverResult { steps: vec![] };
        let qu = vec![100.0, 200.0];
        let qud = vec![80.0, 180.0];
        let ds = vec![0.30, 0.35];
        let fes = vec![1.0, 1.0];
        let heights = vec![3.0, 3.0];
        let result = check_holding_capacity(&pushover, &qu, &qud, &ds, &fes, &heights);
        assert_eq!(result.stories.len(), 2);
        assert!(result.stories[0].ok);
        assert!(result.stories[1].ok);
        assert!((result.stories[0].qun - 24.0).abs() < 1e-9);
        assert!((result.stories[1].qun - 63.0).abs() < 1e-9);
    }

    #[test]
    fn test_check_holding_capacity_ng() {
        let pushover = PushoverResult { steps: vec![] };
        let qu = vec![20.0, 50.0];
        let qud = vec![80.0, 180.0];
        let ds = vec![0.30, 0.35];
        let fes = vec![1.0, 1.0];
        let heights = vec![3.0, 3.0];
        let result = check_holding_capacity(&pushover, &qu, &qud, &ds, &fes, &heights);
        assert!(!result.stories[0].ok);
        assert!(!result.stories[1].ok);
    }
}
