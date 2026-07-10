use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::LoadCombination;

/// 断面検定用の標準荷重組合せを自動生成する。
///
/// RESP-D マニュアル「04 断面検定（許容応力度検定）/ 荷重の組合せ」（構造規定）:
/// - 長期（常時）: G + P
/// - 短期（積雪時）: G + P + S
/// - 短期（地震時）: G + P ± E（正負両加力）
///
/// 地震力は正負の加力ケース（E1/E2 に相当）をそれぞれ組合せに含める。
/// 多雪区域の低減係数（δ1·S 等）・暴風時（W）は荷重ケース未対応のため未生成。
pub fn auto_combinations(
    dl_case: LoadCaseId,
    ll_case: LoadCaseId,
    seismic_x: Option<LoadCaseId>,
    seismic_y: Option<LoadCaseId>,
    snow_case: Option<LoadCaseId>,
) -> Vec<LoadCombination> {
    let mut combos = Vec::new();

    // Long-term (長期): DL + LL
    combos.push(LoadCombination {
        name: "G + P".into(),
        terms: vec![(dl_case, 1.0), (ll_case, 1.0)],
    });

    // Short-term (短期): DL + LL ± Seismic (or + Snow)
    if let Some(sx) = seismic_x {
        combos.push(LoadCombination {
            name: "G + P + Kx".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sx, 1.0)],
        });
        combos.push(LoadCombination {
            name: "G + P - Kx".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sx, -1.0)],
        });
    }
    if let Some(sy) = seismic_y {
        combos.push(LoadCombination {
            name: "G + P + Ky".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sy, 1.0)],
        });
        combos.push(LoadCombination {
            name: "G + P - Ky".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sy, -1.0)],
        });
    }
    if let Some(snow) = snow_case {
        combos.push(LoadCombination {
            name: "G + P + S".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (snow, 1.0)],
        });
    }

    combos
}

/// 荷重組合せ名から断面検定の荷重継続性区分（長期/短期）を判定する。
///
/// RESP-D マニュアル「04 断面検定 / 荷重の組合せ」: G+P のみが長期（常時）、
/// 地震（K/E）・積雪（S）・風（W）を含む組合せは短期。
/// `auto_combinations` の命名規約（"G + P ± Kx" 等）に基づき、
/// 追加項の記号が含まれるかで判定する。
pub fn is_short_term_combo(name: &str) -> bool {
    let upper = name.to_uppercase();
    // "G + P" 以外の項（K: 地震, E: 地震, S: 積雪, W: 風）を含むか
    upper.contains('K') || upper.contains('E') || upper.contains('W') || upper.contains('S')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_combos() {
        let combos = auto_combinations(
            LoadCaseId(1),
            LoadCaseId(2),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        // G+P, G+P±Kx, G+P±Ky の 5 組合せ
        assert_eq!(combos.len(), 5);
        assert_eq!(combos[0].name, "G + P");
        assert_eq!(combos[1].name, "G + P + Kx");
        assert_eq!(combos[2].name, "G + P - Kx");
        assert_eq!(combos[3].name, "G + P + Ky");
        assert_eq!(combos[4].name, "G + P - Ky");
        // 負側加力は係数 -1.0
        assert_eq!(combos[2].terms[2].1, -1.0);
        assert_eq!(combos[4].terms[2].1, -1.0);
    }

    #[test]
    fn test_is_short_term_combo() {
        assert!(!is_short_term_combo("G + P"));
        assert!(is_short_term_combo("G + P + Kx"));
        assert!(is_short_term_combo("G + P - Kx"));
        assert!(is_short_term_combo("G + P + Ky"));
        assert!(is_short_term_combo("G + P + S"));
        assert!(is_short_term_combo("G + P + W"));
        assert!(is_short_term_combo("G + P + E"));
    }
}
