//! PCa（プレキャスト）梁の水平接合面の検討（RESP-D マニュアル「計算編 04
//! 断面検定（許容応力度検定）」鉄筋コンクリート造水平接合面の検討）。
//!
//! 材軸平行接合部（打継ぎ面）のせん断強度が設計用せん断応力度を上回ることを
//! 確認する。使用限界状態・終局限界状態の 2 種類の検討がある。
//!
//! # 位置付け・簡略化
//! - 本モジュールは [`crate::joint`] と同様の**純関数群**であり、断面一次
//!   モーメント Sy・断面二次モーメント I・接合面を横切る補強筋比 p′w などの
//!   入力の組み立ては呼び出し側の責務とする。
//! - モデルに PCa 部材の区分・接合面位置・接合面補強筋のデータ構造が
//!   まだ無いため、モデル走査による自動配線は未実装（属性データモデルの
//!   整備後に joint_wiring と同様の配線を追加する）。
//! - 検定対象位置（鉛直荷重時: 両端上端、地震荷重時: 上端引張となる端部）
//!   の選別は呼び出し側で行う。

use crate::CheckResult;

/// モーメント 2 次曲線分布（RESP-D マニュアル「採用応力」）の M=0 となる
/// 端部からの距離（近い方）[mm]。
///
/// `M(x) = M1 + (−M1 − M2 + 4・M0)・x/L − 4・M0・x²/L²`
///
/// - `m1`, `m2`: 端部モーメント（`m1` は下側引張を正、`m2` は上側引張を正）
/// - `m0`: 単純梁の中央モーメント（下側引張を正）
/// - `l`: 部材長 [mm]
///
/// 終局限界状態の検討の区間長さ Δl の算定（端部から M=0 位置まで）に用いる。
/// 実数解が (0, L) に無い場合は None（全長で同符号のモーメント分布）。
pub fn moment_zero_distance(m1: f64, m2: f64, m0: f64, l: f64) -> Option<f64> {
    if l <= 0.0 {
        return None;
    }
    // M(x) = a・x² + b・x + c、a = −4M0/L²、b = (−M1−M2+4M0)/L、c = M1
    let a = -4.0 * m0 / (l * l);
    let b = (-m1 - m2 + 4.0 * m0) / l;
    let c = m1;
    let roots: Vec<f64> = if a.abs() < 1e-12 {
        if b.abs() < 1e-12 {
            return None;
        }
        vec![-c / b]
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        vec![(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
    };
    // (0, L) 内の解のうち、いずれかの端部に最も近いもの（端部からの距離）。
    roots
        .into_iter()
        .filter(|x| *x > 0.0 && *x < l)
        .map(|x| x.min(l - x))
        .min_by(|p, q| p.partial_cmp(q).unwrap_or(std::cmp::Ordering::Equal))
}

/// PCa 水平接合面の使用限界状態の検討の入力。
pub struct PcaServiceInput {
    /// 部材断面に作用するせん断力 Q [N]。
    pub q: f64,
    /// 水平接合面より外側（断面縁側）のコンクリートの、図心位置からの
    /// 断面一次モーメント Sy [mm³]。
    pub s_y: f64,
    /// 断面二次モーメント I [mm⁴]（応力計算で用いる値と同じ）。
    pub i: f64,
    /// 接合面の幅（梁幅）b [mm]。
    pub b: f64,
    /// 水平接合面の摩擦係数 μ。
    pub mu: f64,
    /// 接合面を横切る補強筋の体積比合計 p′w（= あばら筋 pw + 補強筋 rpw）。
    pub pw_total: f64,
    /// 補強筋の降伏強度 σy [N/mm²]（あばら筋・補強筋で異なる場合は
    /// `p′w・σy = pw・σy + rpw・rσy` となるよう等価値を渡す）。
    pub sigma_y: f64,
}

/// PCa 水平接合面・使用限界状態の検討。
///
/// - 設計用せん断応力度 `τxy = Q・Sy/(b・I)`
/// - せん断強度 `τu = 0.5・μ・p′w・σy`
/// - 検定比 = τxy / τu（1.0 以下で OK）
pub fn pca_horizontal_joint_service(inp: &PcaServiceInput) -> CheckResult {
    let denom = inp.b * inp.i;
    let tau_xy = if denom.abs() > 1e-9 {
        (inp.q * inp.s_y / denom).abs()
    } else {
        f64::INFINITY
    };
    let tau_u = 0.5 * inp.mu * inp.pw_total * inp.sigma_y;
    finish("使用限界", tau_xy, tau_u)
}

/// PCa 水平接合面・終局限界状態の検討。
///
/// - 設計用せん断応力度 `τxy = ΔT/(b・Δl)`
///   - `ΔT`: 区間長さにおいて水平接合面より外側に含まれる引張鉄筋の応力変化量
///     [N]。鉛直荷重に対する検討では `ΔT = Md/(0.9・d)`（Md = α・MDL + β・MLL）、
///     地震時荷重に対する検討では引張鉄筋の降伏耐力（強度倍率考慮）とする
///     （いずれも呼び出し側で算定して渡す）。
///   - `Δl`: 区間長さ [mm]（端部から M=0 位置まで。[`moment_zero_distance`]）。
/// - せん断強度 `τu = μ・p′w・σy`（使用限界の 2 倍＝0.5 係数なし）
/// - 検定比 = τxy / τu（1.0 以下で OK）
pub fn pca_horizontal_joint_ultimate(
    delta_t: f64,
    delta_l: f64,
    b: f64,
    mu: f64,
    pw_total: f64,
    sigma_y: f64,
) -> CheckResult {
    let denom = b * delta_l;
    let tau_xy = if denom.abs() > 1e-9 {
        (delta_t / denom).abs()
    } else {
        f64::INFINITY
    };
    let tau_u = mu * pw_total * sigma_y;
    finish("終局限界", tau_xy, tau_u)
}

fn finish(state: &str, tau_xy: f64, tau_u: f64) -> CheckResult {
    let ratio = if tau_u > 0.0 {
        tau_xy / tau_u
    } else {
        f64::INFINITY
    };
    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis: format!("PCa 水平接合面（{state}状態）せん断検定"),
        detail: format!("τxy={tau_xy:.4} N/mm², τu={tau_u:.4} N/mm², ratio={ratio:.4}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn moment_zero_distance_symmetric_beam() {
        // 両端 M1=M2=-100（上側引張）、中央 M0=+100 の対称分布:
        // M(x) = -100 + 800・x/L − 400・x²/L²…係数確認は解の対称性で行う。
        // M(0)<0, M(L/2)>0 なので (0, L/2) に M=0 があり、対称ゆえ両端から等距離。
        let l = 6000.0;
        let d = moment_zero_distance(-100.0, -100.0, 100.0, l).expect("解があるはず");
        assert!(d > 0.0 && d < l / 2.0);
        // 手計算: -100 + (100+100+400)x/L−400x²/L² = 0 → ξ=x/L:
        // -1 + 6ξ - 4ξ² = 0 → ξ = (6±√(36-16))/8 = (6±√20)/8 → ξ1≈0.1910
        let xi = (6.0 - 20.0_f64.sqrt()) / 8.0;
        assert!((d - xi * l).abs() < 1e-6, "d={d}, expected={}", xi * l);
    }

    #[test]
    fn moment_zero_distance_no_root_when_same_sign() {
        // 全長で正（下側引張のみ）: 根なし。符号規約により M(0)=M1、M(L)=−M2
        // なので、両端で正となるのは M1>0 かつ M2<0 の場合。
        // M(ξ) = 100 + 400ξ − 400ξ²（頂点 ξ=0.5 で 200 > 0）。
        assert!(moment_zero_distance(100.0, -100.0, 100.0, 6000.0).is_none());
    }

    #[test]
    fn pca_service_hand_calc() {
        // 矩形断面 b=400, D=700 の上端から 150mm の接合面:
        // 図心から接合面まで y1 = 350-150 = 200、外側部分 A=400×150、
        // 重心 y=350-75=275 → Sy = 400×150×275 = 16.5e6 mm³。
        // I = 400×700³/12 = 11.433e9 mm⁴。Q=200kN →
        // τxy = 200e3×16.5e6/(400×11.433e9) = 0.7217 N/mm²
        let inp = PcaServiceInput {
            q: 200_000.0,
            s_y: 16.5e6,
            i: 400.0 * 700.0_f64.powi(3) / 12.0,
            b: 400.0,
            mu: 0.6,
            pw_total: 0.008,
            sigma_y: 345.0,
        };
        let res = pca_horizontal_joint_service(&inp);
        let tau_xy = 200_000.0 * 16.5e6 / (400.0 * 400.0 * 700.0_f64.powi(3) / 12.0);
        let tau_u = 0.5 * 0.6 * 0.008 * 345.0;
        assert!((res.ratio - tau_xy / tau_u).abs() < 1e-9);
    }

    #[test]
    fn pca_ultimate_is_twice_service_strength() {
        // 同一 μ・p′w・σy に対し、終局の τu は使用限界の 2 倍。
        let service = pca_horizontal_joint_service(&PcaServiceInput {
            q: 1.0,
            s_y: 1.0,
            i: 1.0,
            b: 1.0,
            mu: 1.0,
            pw_total: 0.01,
            sigma_y: 300.0,
        });
        let ultimate = pca_horizontal_joint_ultimate(1.0, 1.0, 1.0, 1.0, 0.01, 300.0);
        assert!((service.ratio / ultimate.ratio - 2.0).abs() < 1e-9);
    }

    #[test]
    fn pca_zero_strength_is_ng() {
        let res = pca_horizontal_joint_ultimate(1000.0, 100.0, 10.0, 0.6, 0.0, 345.0);
        assert!(!res.ok);
        assert!(res.ratio.is_infinite());
    }
}
