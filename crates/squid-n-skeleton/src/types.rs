//! スケルトン算定で用いるデータ型（部材スケルトン曲線・配筋・制御パラメータ）。
//!
//! このモジュールは純粋なデータ構造のみを保持し、算定ロジックは持たない。

use squid_n_core::model::{Material, Section};
use squid_n_material::HysteresisRule;
use squid_n_section::fiber::FiberSection;

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

impl AxialInteraction {
    /// 軸力依存のない空の相関。
    pub fn empty() -> Self {
        AxialInteraction { skeletons: vec![] }
    }
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

impl MemberSkeleton {
    /// 指定軸力レベルに対する単一スケルトンを軸力依存として登録した曲線を作る。
    ///
    /// `axial_dependency` には `n_axial` に対応する（`points` 空の）補正エントリを 1 つ保持する。
    pub(crate) fn with_axial_entry(
        points: Vec<(f64, f64)>,
        hysteresis: HysteresisRule,
        n_axial: f64,
    ) -> Self {
        MemberSkeleton {
            points,
            hysteresis: hysteresis.clone(),
            axial_dependency: AxialInteraction {
                skeletons: vec![(
                    n_axial,
                    MemberSkeleton {
                        points: vec![],
                        hysteresis,
                        axial_dependency: AxialInteraction::empty(),
                    },
                )],
            },
        }
    }
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
            axial_dependency: AxialInteraction::empty(),
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
