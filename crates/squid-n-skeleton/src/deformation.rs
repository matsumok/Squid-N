//! M–φ（モーメント–曲率）から M–θ（モーメント–部材角）への変換。
//!
//! 責務: 柔性法による曲げ変形に、せん断変形・鉄筋抜出しの寄与を加算して
//! 部材端の回転角を求める。断面積分そのものは [`crate::fiber_model`] が担う。
//!
//! 仕様書 §7 フロー4。各変形成分を加算して部材端 M–θ スケルトンを完成させる。
//!
//! - 曲げ: θ_f = κ·l/3（弾性、曲率分布を三角形と仮定）。降伏後は θ_f = κy·l/3 + (κ-κy)·lp。
//! - せん断: θ_s = M / (K_s · l_eff)。K_s = G·A_w（有効せん断断面積）。
//! - 鉄筋抜出し: θ_p = σ_s · d_b / (E_s · ξ)。ξ=定着区の平均結合応力係数（代表 8〜10）。

use squid_n_material::Concrete;

/// 曲率がゼロとみなせる閾値 [1/mm]。
const CURVATURE_EPS: f64 = 1e-15;

/// M–φ の 1 点を M–θ の 1 点に変換する。
///
/// - `ky`: 曲率 κ [1/mm]
/// - `m`: モーメント M（正で与える）
/// - `ky_yield`: 降伏曲率（塑性ヒンジ判定用。未降伏なら `None`）
/// - `span` × `inflection_ratio`: 反曲点距離 l
/// - `plastic_hinge_length`: 塑性ヒンジ長 lp
pub(crate) fn mphi_to_mtheta(
    ky: f64,
    m: f64,
    ky_yield: Option<f64>,
    span: f64,
    inflection_ratio: f64,
    plastic_hinge_length: f64,
    shear_add: ShearContribution,
    pullout_add: PulloutContribution,
) -> (f64, f64) {
    if ky.abs() < CURVATURE_EPS {
        return (0.0, 0.0);
    }
    let l = span * inflection_ratio;
    // 曲げ変形（降伏後は塑性ヒンジ長で回転を累積）
    let theta_f = match ky_yield {
        Some(ky_y) if ky > ky_y => ky_y * l / 3.0 + (ky - ky_y) * plastic_hinge_length,
        _ => ky * l / 3.0,
    };
    // せん断変形（M から Q=M/l、γ=Q/K_s、θ_s=γ·l）
    let theta_s = shear_add.rotation(m, l);
    // 鉄筋抜出し（κ に対応する鉄筋応力から）
    let theta_p = pullout_add.rotation(ky, ky_yield);
    (theta_f + theta_s + theta_p, m)
}

/// せん断変形の寄与（M-θ への加算分）。
#[derive(Clone, Copy, Debug)]
pub struct ShearContribution {
    /// 等価せん断剛性 K_s = G·A_w [N]。0 なら寄与なし。
    pub k_s: f64,
}

impl ShearContribution {
    pub fn none() -> Self {
        Self { k_s: 0.0 }
    }
    /// RC 矩形断面の等価せん断剛性 G·A_w。A_w = 5/6·b·D（ティモシェンコせん断補正）。
    pub fn rc_rect(width: f64, depth: f64, concrete: &Concrete) -> Self {
        let g = concrete.e0_shear() / (2.0 * (1.0 + 0.2));
        let a_w = 5.0 / 6.0 * width * depth;
        Self { k_s: g * a_w }
    }
    /// Q = M / l（片持ち/逆対称の近似）, γ = Q / K_s, θ_s = γ·l = M/K_s。
    fn rotation(&self, m: f64, l: f64) -> f64 {
        if self.k_s.abs() < 1e-12 || l.abs() < 1e-12 {
            return 0.0;
        }
        m / self.k_s
    }
}

/// 鉄筋抜出しの寄与（M-θ への加算分）。
#[derive(Clone, Copy, Debug)]
pub struct PulloutContribution {
    /// 鉄筋径 d_b [mm]
    pub bar_diameter: f64,
    /// 鉄筋ヤング率 E_s [N/mm²]
    pub e_s: f64,
    /// 降伏強度 f_y [N/mm²]
    pub fy: f64,
    /// 定着区の平均結合応力係数 ξ（代表 8〜10。外部設定）
    pub bond_coeff: f64,
}

impl PulloutContribution {
    pub fn none() -> Self {
        Self {
            bar_diameter: 0.0,
            e_s: 0.0,
            fy: 1.0,
            bond_coeff: 1.0,
        }
    }
    /// κ と降伏曲率 κy から鉄筋応力 σ_s を推定し、θ_p = σ_s·d_b/(E·ξ) を返す。
    /// 弾性域: σ_s ∝ κ/κy · fy。降伏後: σ_s = fy（一定）。
    fn rotation(&self, ky: f64, ky_yield: Option<f64>) -> f64 {
        if self.bar_diameter < 1e-12 || self.e_s < 1e-12 || self.bond_coeff < 1e-12 {
            return 0.0;
        }
        let sigma_s = match ky_yield {
            Some(ky_y) if ky_y.abs() > CURVATURE_EPS => {
                if ky.abs() > ky_y.abs() {
                    self.fy
                } else {
                    (ky / ky_y).abs().min(1.0) * self.fy
                }
            }
            _ => self.fy * 0.5,
        };
        sigma_s * self.bar_diameter / (self.e_s * self.bond_coeff)
    }
}
