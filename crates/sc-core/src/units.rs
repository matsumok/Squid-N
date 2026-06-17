pub const GRAVITY_MM_S2: f64 = 9_806.65;

pub mod to_internal {
    pub fn length_m(m: f64) -> f64 {
        m * 1_000.0
    }
    pub fn force_kn(kn: f64) -> f64 {
        kn * 1_000.0
    }
    pub fn line_load_kn_per_m(v: f64) -> f64 {
        v
    }
    pub fn area_load_kn_per_m2(v: f64) -> f64 {
        v / 1_000.0
    }
    pub fn stress_n_per_mm2(v: f64) -> f64 {
        v
    }
    pub fn mass_density_g_per_cm3(v: f64) -> f64 {
        v * 1.0e-9
    }
    pub fn unit_weight_kn_per_m3(v: f64) -> f64 {
        v * 1.0e-6
    }
    pub fn weight_n_to_mass(w_n: f64) -> f64 {
        w_n / super::GRAVITY_MM_S2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_unit_conversions() {
        assert_relative_eq!(to_internal::length_m(6.0), 6000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::line_load_kn_per_m(10.0),
            10.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(to_internal::force_kn(50.0), 50000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::stress_n_per_mm2(24.0),
            24.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::mass_density_g_per_cm3(2.4),
            2.4e-9,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::unit_weight_kn_per_m3(24.0),
            2.4e-5,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::weight_n_to_mass(1.0e6),
            101.971_621_297_792_82,
            max_relative = 1e-12
        );
    }
}
