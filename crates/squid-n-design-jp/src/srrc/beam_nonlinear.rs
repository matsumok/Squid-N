//! 鉄骨鉄筋コンクリート造梁の**せん断終局強度**（RESP-D マニュアル「計算編 05
//! 非線形モデル」SRC 梁のせん断復元力特性）。
//!
//! # 位置付け
//! 非線形解析のせん断ばね終局耐力を算定する純関数群。RESP-D は 3 式
//! （SRC規準／SRC診断式／構造関係技術基準解説書）から選択できる。本モジュールは
//! 数式が一意な**構造関係技術基準解説書**式と**SRC規準**式を実装する。
//! いずれも RC 部分 rQu と鉄骨部分 sQu の累加 `Qu = rQu + sQu`。
//!
//! # 準拠する規準・出典
//! - 構造関係技術基準解説書 SRC 梁せん断終局。
//! - SRC 規準 SRC 梁せん断終局。
//! いずれも係数は要・原典照合（`specs/原典照合リスト.md`）。

/// SRC 梁せん断終局の算定入力（累加式 Qu=rQu+sQu 共通）。
#[derive(Clone, Copy, Debug)]
pub struct SrcBeamShearInput {
    /// 梁幅 b [mm]。
    pub b: f64,
    /// RC 部分の応力中心間距離 rj [mm]。
    pub rj: f64,
    /// 梁の有効せい rd [mm]。
    pub rd: f64,
    /// せん断補強筋比 rpw（小数）。
    pub rpw: f64,
    /// せん断補強筋の材料強度 rwσy [N/mm²]。
    pub rw_sigma_y: f64,
    /// 鉄骨フランジ位置のコンクリート有効幅 b' [mm]。
    pub b_prime: f64,
    /// せん断力方向の鉄骨ウェブ断面積 sAw [mm²]。
    pub s_aw: f64,
    /// 鉄骨ウェブの材料強度 sσy [N/mm²]。
    pub s_sigma_y: f64,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 鉄骨部分の両端全塑性モーメントの和 ΣsMu [N·mm]（sQu2=ΣsMu/l'）。
    pub sum_s_mu: f64,
    /// 内法スパン l' [mm]。
    pub clear_span: f64,
    /// せん断スパン比 M/(Q·rd)。
    pub m_over_qrd: f64,
}

/// SRC 梁せん断終局強度 Qu [N]（構造関係技術基準解説書式）。
///
/// ```text
/// Qu = rQu + sQu
/// rQu = min(rQu1, rQu2),  sQu = min(sQu1, sQu2)
/// rQu1 = b·rj·(α·fs + 0.5·rpw·rwσy)
/// rQu2 = b·rj·(2·b'/b·fs + rpw·rwσy)
/// sQu1 = sAw·sσy/√3,  sQu2 = ΣsMu/l'
/// fs   = min(Fc/20, (5 + Fc/100)·1.5)
/// α    = 4/(M/(Q·rd)+1)（1≤α≤2 にクランプ）
/// ```
/// 不正入力（b・rj・Fc・l' のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_ultimate_tech_standard(inp: &SrcBeamShearInput) -> f64 {
    if inp.b <= 0.0 || inp.rj <= 0.0 || inp.fc <= 0.0 || inp.clear_span <= 0.0 {
        return 0.0;
    }
    let fs = (inp.fc / 20.0).min((5.0 + inp.fc / 100.0) * 1.5);
    let alpha = (4.0 / (inp.m_over_qrd.max(0.0) + 1.0)).clamp(1.0, 2.0);
    let rqu1 = inp.b * inp.rj * (alpha * fs + 0.5 * inp.rpw * inp.rw_sigma_y);
    let rqu2 = inp.b * inp.rj * (2.0 * inp.b_prime / inp.b * fs + inp.rpw * inp.rw_sigma_y);
    let rqu = rqu1.min(rqu2);
    let squ1 = inp.s_aw * inp.s_sigma_y / 3.0_f64.sqrt();
    let squ2 = inp.sum_s_mu / inp.clear_span;
    let squ = squ1.min(squ2);
    (rqu + squ).max(0.0)
}

/// SRC 梁せん断終局強度 Qu [N]（SRC 規準式）。
///
/// 技術基準解説書式との差異:
/// - `rQu1 = b·rj·(0.5·α·fs + 0.5·rpw·rwσy)`（fs 係数が 0.5·α）
/// - `rQu2 = b·rj·(b'/b·fs + rpw·rwσy)`（b'/b の係数が 1）
/// - `fs = min(0.15·Fc, (22.5 + 4.5·Fc)/100)`（要・原典照合）
///
/// 不正入力（b・rj・Fc・l' のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_ultimate_src_standard(inp: &SrcBeamShearInput) -> f64 {
    if inp.b <= 0.0 || inp.rj <= 0.0 || inp.fc <= 0.0 || inp.clear_span <= 0.0 {
        return 0.0;
    }
    let fs = (0.15 * inp.fc).min((22.5 + 4.5 * inp.fc) / 100.0);
    let alpha = (4.0 / (inp.m_over_qrd.max(0.0) + 1.0)).clamp(1.0, 2.0);
    let rqu1 = inp.b * inp.rj * (0.5 * alpha * fs + 0.5 * inp.rpw * inp.rw_sigma_y);
    let rqu2 = inp.b * inp.rj * (inp.b_prime / inp.b * fs + inp.rpw * inp.rw_sigma_y);
    let rqu = rqu1.min(rqu2);
    let squ1 = inp.s_aw * inp.s_sigma_y / 3.0_f64.sqrt();
    let squ2 = inp.sum_s_mu / inp.clear_span;
    let squ = squ1.min(squ2);
    (rqu + squ).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_input() -> SrcBeamShearInput {
        SrcBeamShearInput {
            b: 500.0,
            rj: 600.0,
            rd: 700.0,
            rpw: 0.004,
            rw_sigma_y: 295.0,
            b_prime: 300.0,
            s_aw: 5000.0,
            s_sigma_y: 235.0,
            fc: 24.0,
            sum_s_mu: 4.0e8,
            clear_span: 6000.0,
            m_over_qrd: 2.0,
        }
    }

    #[test]
    fn test_src_beam_shear_tech_standard_matches_handcalc() {
        let inp = base_input();
        let qu = src_beam_shear_ultimate_tech_standard(&inp);
        let fs = (24.0_f64 / 20.0).min((5.0 + 24.0 / 100.0) * 1.5);
        let alpha = (4.0_f64 / (2.0 + 1.0)).clamp(1.0, 2.0);
        let rqu1 = 500.0 * 600.0 * (alpha * fs + 0.5 * 0.004 * 295.0);
        let rqu2 = 500.0 * 600.0 * (2.0 * 300.0 / 500.0 * fs + 0.004 * 295.0);
        let rqu = rqu1.min(rqu2);
        let squ1 = 5000.0 * 235.0 / 3.0_f64.sqrt();
        let squ2 = 4.0e8 / 6000.0;
        let squ = squ1.min(squ2);
        let hand = rqu + squ;
        assert!((qu - hand).abs() < 1e-3, "Qu={qu} vs {hand}");
        assert!(qu > 0.0);
    }

    #[test]
    fn test_src_beam_shear_src_standard_matches_handcalc() {
        let inp = base_input();
        let qu = src_beam_shear_ultimate_src_standard(&inp);
        let fs = (0.15_f64 * 24.0).min((22.5 + 4.5 * 24.0) / 100.0);
        let alpha = (4.0_f64 / (2.0 + 1.0)).clamp(1.0, 2.0);
        let rqu1 = 500.0 * 600.0 * (0.5 * alpha * fs + 0.5 * 0.004 * 295.0);
        let rqu2 = 500.0 * 600.0 * (300.0 / 500.0 * fs + 0.004 * 295.0);
        let rqu = rqu1.min(rqu2);
        let squ1 = 5000.0 * 235.0 / 3.0_f64.sqrt();
        let squ2 = 4.0e8 / 6000.0;
        let squ = squ1.min(squ2);
        let hand = rqu + squ;
        assert!((qu - hand).abs() < 1e-3, "Qu={qu} vs {hand}");
    }

    #[test]
    fn test_src_beam_shear_alpha_clamped() {
        // M/(Q·rd) 大 → α<1 になるところを 1.0 にクランプ。
        let mut inp = base_input();
        inp.m_over_qrd = 10.0; // 4/11=0.36 → 1.0
        let qu_hi = src_beam_shear_ultimate_tech_standard(&inp);
        inp.m_over_qrd = 3.0; // 4/4=1.0
        let qu_at1 = src_beam_shear_ultimate_tech_standard(&inp);
        assert!((qu_hi - qu_at1).abs() < 1e-6);
    }

    #[test]
    fn test_src_beam_shear_steel_contribution_positive() {
        // 鉄骨ウェブ項が正の寄与（sAw を 0 にすると Qu が下がる）。
        let inp = base_input();
        let mut no_steel = base_input();
        no_steel.s_aw = 0.0;
        no_steel.sum_s_mu = 0.0;
        assert!(
            src_beam_shear_ultimate_tech_standard(&inp)
                > src_beam_shear_ultimate_tech_standard(&no_steel)
        );
    }

    #[test]
    fn test_src_beam_shear_invalid_inputs_zero() {
        let mut bad = base_input();
        bad.fc = 0.0;
        assert_eq!(src_beam_shear_ultimate_tech_standard(&bad), 0.0);
        assert_eq!(src_beam_shear_ultimate_src_standard(&bad), 0.0);
    }
}
