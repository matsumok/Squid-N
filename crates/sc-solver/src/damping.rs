use faer::sparse::SparseColMat;

type SparseMat = SparseColMat<usize, f64>;

/// 減衰モデル（設計書 §10.5 / R9）。
pub enum Damping {
    MassProportional {
        h: f64,
        omega: f64,
    },
    StiffnessProportional {
        h: f64,
        omega: f64,
        basis: StiffnessKind,
    },
    Rayleigh {
        h1: f64,
        w1: f64,
        h2: f64,
        w2: f64,
    },
}

/// 剛性比例減衰がどの剛性を基準にするか。
pub enum StiffnessKind {
    Initial,
    Tangent,
}

impl Damping {
    /// 減衰行列 C を組み立てる。
    #[allow(unused_variables)]
    pub fn assemble_c(&self, m: &SparseMat, k: &SparseMat) -> SparseMat {
        todo!("Damping::assemble_c")
    }

    /// Rayleigh 減衰の係数 (α_m, β_k) を、2つの振動数と目標減衰比から計算する。
    pub fn rayleigh_coeffs(omega1: f64, omega2: f64, h1: f64, h2: f64) -> (f64, f64) {
        let d = omega2 * omega2 - omega1 * omega1;
        let beta_k = 2.0 * (h2 * omega2 - h1 * omega1) / d;
        let alpha_m = 2.0 * omega1 * omega2 * (h1 * omega2 - h2 * omega1) / d;
        (alpha_m, beta_k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_damping_rayleigh_coeffs() {
        let (alpha_m, beta_k) = Damping::rayleigh_coeffs(10.0, 100.0, 0.05, 0.05);
        let omega1 = 10.0;
        let h_actual = (alpha_m / omega1 + beta_k * omega1) / 2.0;
        assert!((h_actual - 0.05).abs() < 1e-6);
    }
}
