use squid_n_core::model::{Material, Section};
use squid_n_material::{Bilinear, Concrete, HysteresisRule, UniaxialMaterial};
use squid_n_section::fiber::{section_response, Fiber, FiberSection, SectionStrain};

/// 配筋情報（RC）。
#[derive(Clone, Debug)]
pub struct Reinforcement {
    /// 主筋の位置（y, z）[mm] と断面積 [mm²] のリスト
    pub main_bars: Vec<(f64, f64, f64)>,
    /// 帯筋ピッチ [mm]
    pub hoop_pitch: f64,
    /// 帯筋1本の断面積 [mm²]
    pub hoop_area: f64,
}

/// N–M 相関情報。
#[derive(Clone, Debug)]
pub struct AxialInteraction {
    /// 複数軸力レベルでのスケルトン
    pub skeletons: Vec<(f64 /* N */, MemberSkeleton)>,
}

/// 部材スケルトン曲線（トリリニア折れ点）。
/// `points` は (変形 θ, 耐力 M) の昇順。`hysteresis` の折点は (耐力 M, 変形 θ) の順。
#[derive(Clone, Debug)]
pub struct MemberSkeleton {
    /// トリリニア折れ点 (変形 θ, 耐力 M)
    pub points: Vec<(f64, f64)>,
    /// 履歴則パラメータ
    pub hysteresis: HysteresisRule,
    /// N によるスケルトン補正
    pub axial_dependency: AxialInteraction,
}

impl Default for MemberSkeleton {
    fn default() -> Self {
        MemberSkeleton {
            points: vec![(0.0, 0.0), (0.01, 10.0), (0.05, 12.0)],
            hysteresis: HysteresisRule::Takeda {
                crack: (1.0, 0.001),
                yield_point: (10.0, 0.01),
                ultimate: (12.0, 0.05),
                alpha: 0.4,
            },
            axial_dependency: AxialInteraction { skeletons: vec![] },
        }
    }
}

/// スケルトン算定の制御パラメータ。
#[derive(Clone, Copy, Debug)]
pub struct SkeletonOptions {
    /// 部材長 [mm]
    pub span: f64,
    /// 反曲点比（M-φ→M-θ 用）
    pub inflection_ratio: f64,
    /// 想定軸力 [N]
    pub n_axial: f64,
    /// 武田モデルの除荷剛性低下指数 α（外部設定。代表 0.4〜0.5）
    pub alpha: f64,
}

/// スケルトン算定に必要な部材情報。
pub struct MemberData<'a> {
    pub section: &'a Section,
    pub reinforcement: &'a Reinforcement,
    pub material: &'a Material,
    pub fibers: &'a FiberSection,
    pub span: f64,
    pub inflection_ratio: f64,
}

/// ファイバの役割（イベント抽出用）。
#[derive(Clone, Copy, Debug, PartialEq)]
enum FiberRole {
    Concrete,
    Steel,
}

/// M–φ 解析のイベント結果。
struct MPhiEvents {
    /// ひび割れ曲率・モーメント
    crack: Option<(f64, f64)>,
    /// 降伏曲率・モーメント
    yield_pt: Option<(f64, f64)>,
    /// 終局（ピークモーメント）曲率・モーメント
    ultimate: Option<(f64, f64)>,
}

/// ファイバ断面から M–φ 関係を数値積分で算定する。
/// 軸力 n_axial [N] を保ちながら曲率 κ [1/mm] を増やし、各ステップでファイバ状態を commit する。
/// ひび割れ（コンクリート引張ひび割れ）・降伏（鉄筋降伏）・終局（ピークモーメント）を検出する。
#[allow(clippy::too_many_arguments)]
fn compute_m_phi_curve_rc(
    fibers: &FiberSection,
    mats: &mut [Box<dyn UniaxialMaterial>],
    roles: &[FiberRole],
    concrete: &Concrete,
    steel: &Bilinear,
    n_axial: f64,
    max_curvature: f64,
    num_steps: usize,
) -> MPhiEvents {
    let e0_conc = 2.0 * concrete.fc / concrete.ec0.abs();
    let eps_cr = concrete.ft / e0_conc;
    let eps_cu = concrete.ecu;
    let eps_y = steel.fy / steel.e;

    let mut crack: Option<(f64, f64)> = None;
    let mut yield_pt: Option<(f64, f64)> = None;
    let mut peak_m = 0.0f64;
    let mut peak_ky = 0.0f64;
    let mut crushed = false;

    for mat in mats.iter_mut() {
        mat.revert();
    }

    let dk = max_curvature / num_steps as f64;
    for i in 0..=num_steps {
        let ky = i as f64 * dk;

        // eps0 をニュートン法で探索（N = n_axial）
        let mut n_iter = 0;
        let mut eps0 = 0.0;
        let max_iter = 50;
        let tol = 1e-6;
        loop {
            let strain = SectionStrain { eps0, ky, kz: 0.0 };
            let (force, _) = section_response(fibers, strain, mats);
            let residual = force.n - n_axial;
            if residual.abs() < n_axial.abs().max(1.0) * tol || n_iter >= max_iter {
                break;
            }
            let strain_p = SectionStrain {
                eps0: eps0 + 1e-8,
                ky,
                kz: 0.0,
            };
            let (force_p, _) = section_response(fibers, strain_p, mats);
            let dn = (force_p.n - force.n) / 1e-8;
            if dn.abs() < 1e-15 {
                break;
            }
            eps0 -= residual / dn;
            n_iter += 1;
        }

        let (force, _) = section_response(fibers, SectionStrain { eps0, ky, kz: 0.0 }, mats);
        let m = force.my.abs();
        if m > peak_m {
            peak_m = m;
            peak_ky = ky;
        }

        // イベント検出（ファイバひずみで判定。kz=0 なので eps = eps0 + ky·z）
        if crack.is_none() || yield_pt.is_none() || !crushed {
            for (j, f) in fibers.fibers.iter().enumerate() {
                let eps = eps0 + ky * f.z;
                match roles[j] {
                    FiberRole::Concrete => {
                        if crack.is_none() && eps > eps_cr {
                            crack = Some((ky, force.my));
                        }
                        if eps < eps_cu {
                            crushed = true;
                        }
                    }
                    FiberRole::Steel => {
                        if yield_pt.is_none() && eps.abs() > eps_y {
                            yield_pt = Some((ky, force.my));
                        }
                    }
                }
            }
        }

        // ファイバ状態をコミット（履歴を進める）
        for mat in mats.iter_mut() {
            mat.commit();
        }

        if crushed && ky > peak_ky {
            break;
        }
    }

    // 終局 = ピークモーメント点
    let ultimate = if peak_m > 0.0 {
        // ピーク時のモーメント符号は force.my に依存するが、skeleton は正で格納
        Some((peak_ky, peak_m))
    } else {
        None
    };

    MPhiEvents {
        crack,
        yield_pt,
        ultimate,
    }
}

/// RC 断面のファイバモデルを構築する。
/// コンクリートは矩形グリッド、主筋は点ファイバ。各ファイバが独自の材料状態を持つ。
/// （主筋位置のコンクリート重複計上は微小な近似。厳密には断面積を控除する。）
fn build_rc_fiber_section(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    reinforcement: &Reinforcement,
    concrete: &Concrete,
    steel: &Bilinear,
) -> (FiberSection, Vec<Box<dyn UniaxialMaterial>>, Vec<FiberRole>) {
    let mut fibers = Vec::new();
    let mut mats: Vec<Box<dyn UniaxialMaterial>> = Vec::new();
    let mut roles = Vec::new();

    // コンクリート格子
    let dw = width / nw as f64;
    let dd = depth / nd as f64;
    for i in 0..nw {
        for j in 0..nd {
            let y = (i as f64 + 0.5) * dw - width / 2.0;
            let z = (j as f64 + 0.5) * dd - depth / 2.0;
            fibers.push(Fiber {
                y,
                z,
                area: dw * dd,
                material: 0,
            });
            mats.push(concrete.clone_box());
            roles.push(FiberRole::Concrete);
        }
    }
    // 主筋（点ファイバ）
    for &(y, z, area) in &reinforcement.main_bars {
        fibers.push(Fiber {
            y,
            z,
            area,
            material: 1,
        });
        mats.push(steel.clone_box());
        roles.push(FiberRole::Steel);
    }
    (FiberSection { fibers }, mats, roles)
}

/// M–φ → M–θ 変換（柔性法＋塑性ヒンジ＋せん断・抜出し・付着すべり）。
/// 仕様書 §7 フロー4。各変形成分を加算して部材端 M–θ スケルトンを完成させる。
///
/// - 曲げ: θ_f = κ·l/3（弾性、曲率分布を三角形と仮定）。降伏後は θ_f = κy·l/3 + (κ-κy)·lp。
/// - せん断: θ_s = M / (K_s · l_eff)。K_s = G·A_w（有効せん断断面積）。l_eff = l（反曲点距離）。
///   M-φ 積分で得た M を用いて Q = M/l から γ、θ_s = γ·l を算出する簡易形。
/// - 鉄筋抜出し: θ_p = σ_s · d_b / (E_s · ξ)。ξ=定着区の平均結合応力係数（代表 8〜10）。
///   σ_s は鉄筋応力（κ に対応）。降伏時 σ_s=fy → θ_p,y = fy·d_b/(E·ξ)。
/// - 付着すべり: 降伏後のすべしは塑性ヒンジの回転に含める（簡易: θ_p を降伏後一定とする）。
#[allow(clippy::too_many_arguments)]
fn mphi_to_mtheta(
    ky: f64,
    m: f64,
    ky_yield: Option<f64>,
    span: f64,
    inflection_ratio: f64,
    plastic_hinge_length: f64,
    shear_add: ShearContribution,
    pullout_add: PulloutContribution,
) -> (f64, f64) {
    if ky.abs() < 1e-15 {
        return (0.0, 0.0);
    }
    let l = span * inflection_ratio;
    // 曲げ変形
    let theta_f = if let Some(ky_y) = ky_yield {
        if ky > ky_y {
            ky_y * l / 3.0 + (ky - ky_y) * plastic_hinge_length
        } else {
            ky * l / 3.0
        }
    } else {
        ky * l / 3.0
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
    fn rotation(&self, m: f64, l: f64) -> f64 {
        if self.k_s.abs() < 1e-12 || l.abs() < 1e-12 {
            return 0.0;
        }
        // Q = M / l（片持ち/逆対称の近似）, γ = Q / K_s, θ_s = γ·l = M/K_s
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
            Some(ky_y) if ky_y.abs() > 1e-15 => {
                let ratio = (ky / ky_y).abs().min(1.0);
                if ky.abs() > ky_y.abs() {
                    self.fy
                } else {
                    ratio * self.fy
                }
            }
            _ => self.fy * 0.5,
        };
        sigma_s * self.bar_diameter / (self.e_s * self.bond_coeff)
    }
}

/// RC 部材スケルトンを構築する（仕様書 §7）。
///
/// 1. コンクリート格子＋主筋点ファイバのファイバ断面を構築。
/// 2. 軸力 n_axial を保ちながら M–φ を数値積分。ひび割れ・鉄筋降伏・コンクリート圧壊を
///    ひずみイベントで検出しトリリニア折点とする（規準式準拠のイベント駆動）。
/// 3. 反曲点比・塑性ヒンジ長で M–φ → M–θ へ変換。せん断変形・鉄筋抜出しを加算（§7 フロー4）。
///
/// 単位: 断面寸法・位置 [mm], 面積 [mm²], 軸力 [N], モーメント [N·mm], スパン [mm]。
pub fn build_rc_member_skeleton(
    section: &Section,
    reinforcement: &Reinforcement,
    concrete: &Concrete,
    steel: &Bilinear,
    opts: &SkeletonOptions,
    shear: &ShearContribution,
    pullout: &PulloutContribution,
) -> MemberSkeleton {
    let span = opts.span;
    let inflection_ratio = opts.inflection_ratio;
    let n_axial = opts.n_axial;
    let nw = 16;
    let nd = 32;
    let plastic_hinge_length = 0.5 * section.depth;

    // スイープ範囲を断面・材料から適応的に決定（降伏・終局曲率を十分に解像する）。
    let e0_conc = 2.0 * concrete.fc / concrete.ec0.abs();
    let eps_cr = concrete.ft / e0_conc;
    let eps_y = steel.fy / steel.e;
    let d_eff = section.depth
        - reinforcement
            .main_bars
            .iter()
            .map(|(_, z, _)| (section.depth / 2.0 - z).abs())
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(section.depth / 2.0);
    let j = 7.0 * d_eff / 8.0;
    let ky_yield_est = eps_y / j;
    let ky_ultimate_est = concrete.ecu.abs() / (section.depth / 2.0);
    let max_curvature = (3.0 * ky_ultimate_est).max(2.0 * ky_yield_est).max(1e-4);
    let num_steps = 800;

    let (fibers, mut mats, roles) = build_rc_fiber_section(
        section.width,
        section.depth,
        nw,
        nd,
        reinforcement,
        concrete,
        steel,
    );

    let events = compute_m_phi_curve_rc(
        &fibers,
        &mut mats,
        &roles,
        concrete,
        steel,
        n_axial,
        max_curvature,
        num_steps,
    );

    // イベントから折点を取り出す。検出されなかった場合は推定曲率で補完。
    let ky_y_est = ky_yield_est;
    let (ky_crack, m_c) = events
        .crack
        .unwrap_or((eps_cr / (section.depth / 2.0), 0.0));
    let (ky_yield, m_y) = events
        .yield_pt
        .unwrap_or((ky_yield_est, section.iy * e0_conc * ky_yield_est));
    let (ky_ultimate, m_u) = events.ultimate.unwrap_or((ky_ultimate_est, m_y * 1.2));
    let _ = ky_y_est;

    let (theta_c, _) = mphi_to_mtheta(
        ky_crack,
        m_c.abs(),
        Some(ky_yield),
        span,
        inflection_ratio,
        plastic_hinge_length,
        *shear,
        *pullout,
    );
    let (theta_y, _) = mphi_to_mtheta(
        ky_yield,
        m_y.abs(),
        Some(ky_yield),
        span,
        inflection_ratio,
        plastic_hinge_length,
        *shear,
        *pullout,
    );
    let (theta_u, _) = mphi_to_mtheta(
        ky_ultimate,
        m_u.abs(),
        Some(ky_yield),
        span,
        inflection_ratio,
        plastic_hinge_length,
        *shear,
        *pullout,
    );

    let m_c = m_c.abs();
    let m_y = m_y.abs();
    let m_u = m_u.abs();
    let points = vec![(0.0, 0.0), (theta_c, m_c), (theta_y, m_y), (theta_u, m_u)];

    let hysteresis = HysteresisRule::Takeda {
        crack: (m_c, theta_c),
        yield_point: (m_y, theta_y),
        ultimate: (m_u, theta_u),
        alpha: opts.alpha,
    };

    MemberSkeleton {
        points,
        hysteresis: hysteresis.clone(),
        axial_dependency: AxialInteraction {
            skeletons: vec![(
                n_axial,
                MemberSkeleton {
                    points: vec![],
                    hysteresis,
                    axial_dependency: AxialInteraction { skeletons: vec![] },
                },
            )],
        },
    }
}

/// 既定のファイバ断面（呼出側提供）からスケルトンを構築する（汎用パス）。
/// `mats.len() == member.fibers.fibers.len()` が必要（ファイバごとの独立状態）。
/// RC の場合は `build_rc_member_skeleton` を用いること。
/// `alpha` は武田モデルの除荷剛性低下指数（外部設定。代表 0.4〜0.5）。
pub fn build_member_skeleton(
    member: &MemberData,
    n_axial: f64,
    mats: &mut [Box<dyn UniaxialMaterial>],
    alpha: f64,
) -> MemberSkeleton {
    assert_eq!(
        mats.len(),
        member.fibers.fibers.len(),
        "build_member_skeleton: mats.len() must equal fibers.len() (per-fiber state)"
    );
    let max_curvature = 0.01;
    let num_steps = 200;
    let plastic_hinge_length = 0.5 * member.section.depth;

    let mut points = Vec::with_capacity(num_steps + 1);
    for mat in mats.iter_mut() {
        mat.revert();
    }
    let dk = max_curvature / num_steps as f64;
    for i in 0..=num_steps {
        let ky = i as f64 * dk;
        let mut eps0 = 0.0;
        for _it in 0..50 {
            let strain = SectionStrain { eps0, ky, kz: 0.0 };
            let (force, _) = section_response(member.fibers, strain, mats);
            let residual = force.n - n_axial;
            if residual.abs() < n_axial.abs().max(1.0) * 1e-6 {
                break;
            }
            let strain_p = SectionStrain {
                eps0: eps0 + 1e-8,
                ky,
                kz: 0.0,
            };
            let (force_p, _) = section_response(member.fibers, strain_p, mats);
            let dn = (force_p.n - force.n) / 1e-8;
            if dn.abs() < 1e-15 {
                break;
            }
            eps0 -= residual / dn;
        }
        let strain = SectionStrain { eps0, ky, kz: 0.0 };
        let (force, _) = section_response(member.fibers, strain, mats);
        for m in mats.iter_mut() {
            m.commit();
        }
        points.push((ky, force.my));
    }

    // 折点: M-φ 曲線の勾配変化を簡易抽出（汎用パス。RC は build_rc を使用）
    let trilinear = extract_trilinear_generic(&points);
    let ky_y = trilinear.get(2).map(|p| p.0);
    let mtheta: Vec<(f64, f64)> = trilinear
        .iter()
        .map(|&(ky, m)| {
            mphi_to_mtheta(
                ky,
                m,
                ky_y,
                member.span,
                member.inflection_ratio,
                plastic_hinge_length,
                ShearContribution::none(),
                PulloutContribution::none(),
            )
        })
        .collect();

    let hysteresis = HysteresisRule::Takeda {
        crack: (
            mtheta.get(1).map(|p| p.1).unwrap_or(0.0),
            mtheta.get(1).map(|p| p.0).unwrap_or(0.0),
        ),
        yield_point: (
            mtheta.get(2).map(|p| p.1).unwrap_or(0.0),
            mtheta.get(2).map(|p| p.0).unwrap_or(0.0),
        ),
        ultimate: (
            mtheta.last().map(|p| p.1).unwrap_or(0.0),
            mtheta.last().map(|p| p.0).unwrap_or(0.0),
        ),
        alpha,
    };

    MemberSkeleton {
        points: mtheta,
        hysteresis: hysteresis.clone(),
        axial_dependency: AxialInteraction {
            skeletons: vec![(
                n_axial,
                MemberSkeleton {
                    points: vec![],
                    hysteresis,
                    axial_dependency: AxialInteraction { skeletons: vec![] },
                },
            )],
        },
    }
}

/// 汎用パスの折点抽出（勾配ヒューリスティック。RC には build_rc を用いること）。
fn extract_trilinear_generic(mphi: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if mphi.is_empty() {
        return vec![(0.0, 0.0)];
    }
    let n = mphi.len();
    let crack_idx = mphi
        .iter()
        .position(|(_, m)| m.abs() > 0.0)
        .unwrap_or(1)
        .max(1);
    let init_slope = if mphi[crack_idx].0.abs() > 1e-15 {
        mphi[crack_idx].1 / mphi[crack_idx].0
    } else {
        mphi[1].1 / mphi[1].0
    };
    let yield_idx = (crack_idx + 1..n)
        .find(|&i| {
            let dmdk = (mphi[i].1 - mphi[i - 1].1) / (mphi[i].0 - mphi[i - 1].0).max(1e-15);
            init_slope > 0.0 && dmdk / init_slope < 0.3
        })
        .unwrap_or(n - 1);
    let ultimate_idx = n - 1;
    vec![
        (0.0, 0.0),
        (mphi[crack_idx.min(n - 1)].0, mphi[crack_idx.min(n - 1)].1),
        (mphi[yield_idx].0, mphi[yield_idx].1),
        (mphi[ultimate_idx].0, mphi[ultimate_idx].1),
    ]
}

#[cfg(test)]
mod tests;
