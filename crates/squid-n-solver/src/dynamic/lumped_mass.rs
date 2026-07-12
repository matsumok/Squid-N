//! 質点系（串団子）モデルの生成（RESP-D「07 非線形解析（動的解析）」質点系解析モデル
//! の非線形特性）。
//!
//! 立体フレームのプッシュオーバー（漸増静的）結果から、層ごとの層せん断力 Q・層間変形 δ
//! 関係（Q-δ 曲線）を抽出し、**等包絡面積則**でトリリニア骨格へ縮約した串団子モデルを
//! 生成する。
//!
//! - 初期剛性 K1: プッシュオーバー第1ステップの荷重-変形勾配。
//! - 第3折点（終局）: Q-δ 曲線の終端。第3勾配 K3: 終端の接線勾配。
//! - 第1折点: 割線剛性が K1 の指定比率以下となった変位（ルール1）、第1勾配は K1。
//! - 第2折点: 0→第3折点の包絡面積が実曲線と等しくなるよう自動決定。
//!
//! 詳細なルール1/2/3の分岐（降伏部材比率等）は簡略化しており、第1折点の判定は
//! 割線剛性比率（`secant_ratio`）で行う。

use crate::pushover::PushoverResult;
use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;
use squid_n_core::units::GRAVITY_MM_S2;

/// 層のトリリニア骨格（Q-δ）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoryTrilinear {
    /// 初期剛性 K1 [N/mm]。
    pub k1: f64,
    /// 第1折点 (δ1[mm], Q1[N])。
    pub d1: f64,
    pub q1: f64,
    /// 第2折点 (δ2, Q2)。
    pub d2: f64,
    pub q2: f64,
    /// 第3折点＝終局 (δ3, Q3)。
    pub d3: f64,
    pub q3: f64,
}

impl StoryTrilinear {
    /// 第2勾配 K2 = (Q2−Q1)/(δ2−δ1)。
    pub fn k2(&self) -> f64 {
        if self.d2 > self.d1 {
            (self.q2 - self.q1) / (self.d2 - self.d1)
        } else {
            0.0
        }
    }
    /// 第3勾配 K3 = (Q3−Q2)/(δ3−δ2)。
    pub fn k3(&self) -> f64 {
        if self.d3 > self.d2 {
            (self.q3 - self.q2) / (self.d3 - self.d2)
        } else {
            0.0
        }
    }
}

/// 串団子モデルの1質点（層）。
#[derive(Clone, Copy, Debug)]
pub struct StoryStick {
    pub story: StoryId,
    /// 質量 [t]（= 地震重量 W / g）。
    pub mass: f64,
    /// 階高 [mm]。
    pub height: f64,
    /// 層の復元力特性（トリリニア）。
    pub skeleton: StoryTrilinear,
}

/// モデル化タイプ（RESP-D「07」モデル化タイプ）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LumpedMassType {
    /// 等価せん断型（曲げ剛性を剛とする）。
    #[default]
    EquivalentShear,
    /// 等価曲げせん断型（曲げ剛性を梁要素として考慮）。
    EquivalentBendingShear,
    /// 曲げせん断分離型（曲げ剛性を回転ばねとして考慮）。
    BendingShearSeparated,
}

impl LumpedMassType {
    pub fn label(&self) -> &'static str {
        match self {
            LumpedMassType::EquivalentShear => "等価せん断型",
            LumpedMassType::EquivalentBendingShear => "等価曲げせん断型",
            LumpedMassType::BendingShearSeparated => "曲げせん断分離型",
        }
    }
}

/// 串団子モデル。層ごとの質点と復元力特性を保持する。
pub struct LumpedMassModel {
    pub model_type: LumpedMassType,
    pub stories: Vec<StoryStick>,
}

/// 台形則で (0,0) から曲線終端までの包絡面積を求める。
fn envelope_area(pts: &[(f64, f64)]) -> f64 {
    let mut a = 0.0;
    let (mut pd, mut pq) = (0.0, 0.0);
    for &(d, q) in pts {
        a += 0.5 * (pq + q) * (d - pd);
        pd = d;
        pq = q;
    }
    a
}

/// 層 Q-δ 曲線（δ 昇順・正値）を等包絡面積則でトリリニアへ縮約する。
/// `secant_ratio`（0..1）: 第1折点＝割線剛性が K1 のこの比率以下となる変位。
pub fn fit_story_trilinear(curve: &[(f64, f64)], secant_ratio: f64) -> StoryTrilinear {
    // 正の変形のみ・δ 昇順に整える。
    let mut pts: Vec<(f64, f64)> = curve.iter().copied().filter(|&(d, _)| d > 0.0).collect();
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);

    if pts.is_empty() {
        return StoryTrilinear {
            k1: 0.0,
            d1: 0.0,
            q1: 0.0,
            d2: 0.0,
            q2: 0.0,
            d3: 0.0,
            q3: 0.0,
        };
    }
    let (d_first, q_first) = pts[0];
    let (d3, q3) = *pts.last().unwrap();
    let k1 = if d_first > 0.0 {
        q_first / d_first
    } else {
        0.0
    };
    if k1 <= 0.0 || d3 <= d_first {
        // 単調1点・剛性不定は弾性トリリニア（折点なし）で返す。
        return StoryTrilinear {
            k1,
            d1: d3,
            q1: q3,
            d2: d3,
            q2: q3,
            d3,
            q3,
        };
    }
    // 第3勾配 K3 = 終端接線（[0, K1] にクランプ）。
    let k3 = if pts.len() >= 2 {
        let (dp, qp) = pts[pts.len() - 2];
        if d3 > dp {
            ((q3 - qp) / (d3 - dp)).clamp(0.0, k1)
        } else {
            0.0
        }
    } else {
        (q3 / d3).clamp(0.0, k1)
    };
    // 第1折点 δ1: 接線勾配が secant_ratio·K1 を初めて下回る直前の変位（弾性限）。
    // 第1勾配は K1。接線基準は割線基準より弾性限（折れ点）を鋭く捉える（降伏後剛性が
    // 小さい場合でも Q1=K1·δ1 が過大にならない）。
    let thr = secant_ratio * k1;
    let mut d1 = d3 * 0.5;
    let mut prev = (0.0, 0.0);
    let mut found = false;
    for &(d, q) in &pts {
        let tan = if d > prev.0 {
            (q - prev.1) / (d - prev.0)
        } else {
            k1
        };
        if tan < thr && prev.0 > 0.0 {
            d1 = prev.0;
            found = true;
            break;
        }
        prev = (d, q);
    }
    if !found {
        d1 = d3 * 0.5;
    }
    let d1 = d1.clamp(d_first, d3 * 0.9);
    let q1 = k1 * d1;

    // 等包絡面積: A_tri(δ2)=A_actual を解く。Q2 は第3勾配直線上 Q2=Q3−K3(δ3−δ2)。
    // A_tri は δ2 について線形（∂A/∂δ2 = ½[(Q1−Q3)+K3(δ3−δ1)] 一定）なので直接解ける。
    let a_actual = envelope_area(&pts);
    let a_tri = |d2: f64| {
        let q2 = q3 - k3 * (d3 - d2);
        0.5 * d1 * q1 + 0.5 * (q1 + q2) * (d2 - d1) + 0.5 * (q2 + q3) * (d3 - d2)
    };
    let slope = 0.5 * ((q1 - q3) + k3 * (d3 - d1));
    let d2 = if slope.abs() < 1e-30 {
        0.5 * (d1 + d3)
    } else {
        (d1 + (a_actual - a_tri(d1)) / slope).clamp(d1, d3)
    };
    let q2 = q3 - k3 * (d3 - d2);

    StoryTrilinear {
        k1,
        d1,
        q1,
        d2,
        q2,
        d3,
        q3,
    }
}

/// プッシュオーバー結果から串団子モデル（層ごとの質点・復元力特性）を生成する。
/// `secant_ratio`: 第1折点判定の割線剛性比（既定 0.75 程度）。
pub fn build_lumped_mass_model(
    model: &Model,
    pushover: &PushoverResult,
    model_type: LumpedMassType,
    secant_ratio: f64,
) -> LumpedMassModel {
    let n_story = model.stories.len();
    let mut sticks = Vec::with_capacity(n_story);
    for (i, story) in model.stories.iter().enumerate() {
        // 層 i の Q-δ 曲線（各キャパシティ点の層せん断・層間変形）。
        let curve: Vec<(f64, f64)> = pushover
            .capacity_curve
            .iter()
            .filter_map(|cp| {
                let d = cp.story_drift.get(i).copied()?.abs();
                let q = cp.story_shear.get(i).copied()?.abs();
                Some((d, q))
            })
            .collect();
        let skeleton = fit_story_trilinear(&curve, secant_ratio);

        // 質量 = 地震重量 / g（未設定なら節点質量の合計）。
        let mass = match story.seismic_weight {
            Some(w) if w > 0.0 => w / GRAVITY_MM_S2,
            _ => story
                .node_ids
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .filter_map(|n| n.mass)
                .map(|m| m[0].max(m[1]))
                .sum(),
        };
        // 階高 = 当該階標高 − 直下階標高（最下階は標高そのもの）。
        let below = if i > 0 {
            model.stories[i - 1].elevation
        } else {
            0.0
        };
        let height = (story.elevation - below).max(0.0);

        sticks.push(StoryStick {
            story: story.id,
            mass,
            height,
            skeleton,
        });
    }
    LumpedMassModel {
        model_type,
        stories: sticks,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fit_trilinear_equal_area_and_endpoints() {
        // 実曲線: 折れ点のあるなめらかな軟化曲線を細かくサンプル。
        // 0→(1,100) K1=100、(1,100)→(3,140) K2=20、(3,140)→(6,155) K3=5。
        let mut curve = Vec::new();
        for step in 1..=60 {
            let d = step as f64 * 0.1;
            let q = if d <= 1.0 {
                100.0 * d
            } else if d <= 3.0 {
                100.0 + 20.0 * (d - 1.0)
            } else {
                140.0 + 5.0 * (d - 3.0)
            };
            curve.push((d, q));
        }
        let tri = fit_story_trilinear(&curve, 0.9);
        // K1 = 初期剛性 100。
        assert!((tri.k1 - 100.0).abs() < 1.0, "k1={}", tri.k1);
        // 終端 (6, 155)。
        assert!((tri.d3 - 6.0).abs() < 1e-6 && (tri.q3 - 155.0).abs() < 1e-6);
        // 折点は昇順・耐力単調増加。
        assert!(tri.d1 < tri.d2 && tri.d2 <= tri.d3);
        assert!(tri.q1 <= tri.q2 + 1e-9 && tri.q2 <= tri.q3 + 1e-9);
        // 等包絡面積: トリリニアの面積 = 実曲線の面積。
        let a_actual = envelope_area(&curve);
        let a_tri = 0.5 * tri.d1 * tri.q1
            + 0.5 * (tri.q1 + tri.q2) * (tri.d2 - tri.d1)
            + 0.5 * (tri.q2 + tri.q3) * (tri.d3 - tri.d2);
        assert!(
            (a_tri - a_actual).abs() < 1e-3 * a_actual,
            "equal-area: a_tri={a_tri}, a_actual={a_actual}"
        );
    }

    #[test]
    fn test_fit_trilinear_k2_k3_helpers() {
        // 3勾配（K1=80 > K2=30 > K3=8）の軟化曲線。
        let curve: Vec<(f64, f64)> = (1..=50)
            .map(|s| {
                let d = s as f64 * 0.1;
                let q = if d <= 1.0 {
                    80.0 * d
                } else if d <= 2.5 {
                    80.0 + 30.0 * (d - 1.0)
                } else {
                    125.0 + 8.0 * (d - 2.5)
                };
                (d, q)
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!(
            tri.d1 < tri.d2 && tri.d2 < tri.d3,
            "distinct folds: {tri:?}"
        );
        assert!(
            tri.k1 >= tri.k2() && tri.k2() >= tri.k3() - 1e-6,
            "K1>=K2>=K3: k1={}, k2={}, k3={}",
            tri.k1,
            tri.k2(),
            tri.k3()
        );
        assert!(tri.k3() >= 0.0 && tri.k3() <= tri.k1);
    }

    #[test]
    fn test_fit_trilinear_bilinear_input_reduces_gracefully() {
        // バイリニア入力（K1=50→K=5）はトリリニアが縮退（d1≈d2）しても panic せず妥当。
        let curve: Vec<(f64, f64)> = (1..=30)
            .map(|s| {
                let d = s as f64 * 0.1;
                (d, 50.0 * d.min(2.0) + 5.0 * (d - 2.0).max(0.0))
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!((tri.k1 - 50.0).abs() < 1.0);
        assert!(tri.d1 <= tri.d2 && tri.d2 <= tri.d3);
        assert!((tri.d3 - 3.0).abs() < 1e-6 && (tri.q3 - 105.0).abs() < 1e-6);
    }

    #[test]
    fn test_fit_trilinear_empty_and_degenerate() {
        let tri = fit_story_trilinear(&[], 0.75);
        assert_eq!(tri.k1, 0.0);
        // 1点のみ（弾性）。
        let tri1 = fit_story_trilinear(&[(2.0, 200.0)], 0.75);
        assert!((tri1.k1 - 100.0).abs() < 1e-9);
    }
}
