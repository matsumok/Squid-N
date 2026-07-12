//! 部材スケルトン曲線（トリリニア）の算定。
//!
//! RC 部材のファイバ断面から M–φ（モーメント–曲率）を数値積分し、反曲点比・
//! 塑性ヒンジ・せん断変形・鉄筋抜出しを考慮して M–θ（モーメント–部材角）の
//! トリリニアスケルトンと武田履歴則を構築する（仕様書 §7）。
//!
//! モジュール構成（責務分離）:
//! - [`types`]: データ型（[`MemberSkeleton`] / [`Reinforcement`] など）。
//! - [`fiber_model`]: RC ファイバ断面の生成と M–φ 数値積分（内部）。
//! - [`deformation`]: M–φ → M–θ 変換とせん断・抜出し寄与。
//! - [`builder`]: 公開ビルダ [`build_rc_member_skeleton`] / [`build_member_skeleton`]。

mod builder;
mod deformation;
mod fiber_model;
mod types;

pub use builder::{build_member_skeleton, build_rc_member_skeleton};
pub use deformation::{PulloutContribution, ShearContribution};
pub use types::{AxialInteraction, MemberData, MemberSkeleton, Reinforcement, SkeletonOptions};

#[cfg(test)]
mod tests;
