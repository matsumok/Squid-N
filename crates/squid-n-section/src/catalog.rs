//! 日本国内の鋼材断面カタログ（JIS 規格等）。
//!
//! `data/japan_steel_sections.csv` をビルド時に埋め込み、初回アクセス時に一度だけ
//! パースしてキャッシュする。対象国は日本のみのため、CSV の `country` 列は UI に
//! 出さない。利用側は [`CatalogShape`]（大分類）→ `family`（まとまり）→
//! [`CatalogEntry`]（断面名）の3段階で選び、[`to_section`] で `Section` を生成する。

use squid_n_core::ids::SectionId;
use squid_n_core::model::Section;
use std::sync::OnceLock;

const CSV_DATA: &str = include_str!("../data/japan_steel_sections.csv");

/// カタログの大分類（CSV の `shape` 列に対応）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CatalogShape {
    /// H形鋼
    H,
    /// 角形鋼管
    Box,
    /// 円形鋼管
    Pipe,
    /// フラットバー
    Flat,
}

impl CatalogShape {
    pub fn label(self) -> &'static str {
        match self {
            CatalogShape::H => "H形鋼",
            CatalogShape::Box => "角形鋼管",
            CatalogShape::Pipe => "円形鋼管",
            CatalogShape::Flat => "フラットバー",
        }
    }

    pub const ALL: [CatalogShape; 4] = [
        CatalogShape::H,
        CatalogShape::Box,
        CatalogShape::Pipe,
        CatalogShape::Flat,
    ];

    fn from_csv_code(code: &str) -> Option<Self> {
        match code {
            "H" => Some(CatalogShape::H),
            "[]" => Some(CatalogShape::Box),
            "O" => Some(CatalogShape::Pipe),
            "V" => Some(CatalogShape::Flat),
            _ => None,
        }
    }
}

/// カタログ1件分の断面諸元（mm 系単位に変換済み）。
#[derive(Debug, Clone, PartialEq)]
pub struct CatalogEntry {
    pub shape: CatalogShape,
    /// CSV の `family` 列（断面の「まとまり」。例: "H(const)", "BCP" など）。
    pub family: String,
    /// CSV の `name` 列（例: "H-400x200x9x12x13"）。
    pub name: String,
    /// せい／外径 [mm]
    pub depth: f64,
    /// 幅 [mm]（丸鋼管は外径と同じ）
    pub width: f64,
    /// 断面積 [mm²]
    pub area: f64,
    /// せん断断面積（強軸側）[mm²]
    pub as_y: f64,
    /// せん断断面積（弱軸側）[mm²]
    pub as_z: f64,
    /// 強軸断面二次モーメント [mm⁴]
    pub iy: f64,
    /// 弱軸断面二次モーメント [mm⁴]
    pub iz: f64,
    /// ねじり定数 [mm⁴]
    pub j: f64,
}

/// カタログ全件（初回アクセス時にパースしキャッシュする）。
pub fn entries() -> &'static [CatalogEntry] {
    static CACHE: OnceLock<Vec<CatalogEntry>> = OnceLock::new();
    CACHE.get_or_init(parse_csv)
}

/// 指定した大分類に属する family（まとまり）一覧を、CSV 出現順を保って重複なく返す。
pub fn families(shape: CatalogShape) -> Vec<&'static str> {
    let mut out: Vec<&'static str> = Vec::new();
    for e in entries() {
        if e.shape == shape && !out.contains(&e.family.as_str()) {
            out.push(e.family.as_str());
        }
    }
    out
}

/// 指定した大分類・family に属する断面一覧（CSV 出現順）。
pub fn entries_in(shape: CatalogShape, family: &str) -> Vec<&'static CatalogEntry> {
    entries()
        .iter()
        .filter(|e| e.shape == shape && e.family == family)
        .collect()
}

/// カタログ断面から `Section` を生成する。
pub fn to_section(entry: &CatalogEntry, id: SectionId) -> Section {
    Section {
        id,
        name: entry.name.clone(),
        area: entry.area,
        iy: entry.iy,
        iz: entry.iz,
        j: entry.j,
        depth: entry.depth,
        width: entry.width,
        as_y: entry.as_y,
        as_z: entry.as_z,
        panel_thickness: None,
        thickness: None,
    }
}

fn parse_csv() -> Vec<CatalogEntry> {
    let mut out = Vec::new();
    // 1行目: ヘッダ名, 2行目: 単位。データは3行目から。
    for line in CSV_DATA.lines().skip(2) {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(';').map(|f| f.trim_matches('"')).collect();
        // It (ねじり定数) は index 28。それ未満しか無い行は不正なので無視する。
        if fields.len() <= 28 {
            continue;
        }
        let Some(shape) = CatalogShape::from_csv_code(fields[4]) else {
            continue;
        };
        let parse = |s: &str| s.trim().parse::<f64>().unwrap_or(0.0);
        let h = parse(fields[5]);
        let b_upper = parse(fields[7]);
        let area = parse(fields[16]) * 100.0; // cm² -> mm²
        let a_y = parse(fields[17]) * 100.0;
        let a_z = parse(fields[18]) * 100.0;
        let iy = parse(fields[19]) * 1.0e4; // cm⁴ -> mm⁴
        let iz = parse(fields[24]) * 1.0e4;
        let it = parse(fields[28]) * 1.0e4;
        // 丸鋼管は b_upper 列が空欄のため、幅は外径（h）を流用する。
        let width = if matches!(shape, CatalogShape::Pipe) {
            h
        } else {
            b_upper
        };
        out.push(CatalogEntry {
            shape,
            family: fields[2].to_string(),
            name: fields[3].to_string(),
            depth: h,
            width,
            area,
            as_y: a_y.abs(),
            as_z: a_z.abs(),
            iy: iy.abs(),
            iz: iz.abs(),
            j: it.abs(),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_catalog_loads_japan_only_shapes() {
        let all = entries();
        assert!(!all.is_empty());
        // 4大分類（H形鋼／角形鋼管／円形鋼管／フラットバー）が揃っている
        for shape in CatalogShape::ALL {
            assert!(
                all.iter().any(|e| e.shape == shape),
                "missing shape {:?}",
                shape
            );
        }
    }

    #[test]
    fn test_families_no_duplicates_and_preserve_order() {
        let fams = families(CatalogShape::H);
        assert!(fams.contains(&"H(const)"));
        let mut seen = std::collections::HashSet::new();
        for f in &fams {
            assert!(seen.insert(*f), "duplicate family {f}");
        }
    }

    #[test]
    fn test_entries_in_h_const_contains_known_section() {
        let list = entries_in(CatalogShape::H, "H(const)");
        assert!(list.iter().any(|e| e.name == "H-400x200x9x12x13"));
    }

    #[test]
    fn test_h_400x200x9x12x13_matches_known_properties() {
        let list = entries_in(CatalogShape::H, "H(const)");
        let e = list
            .iter()
            .find(|e| e.name == "H-400x200x9x12x13")
            .expect("section not found");
        assert!((e.depth - 400.0).abs() < 1e-6);
        assert!((e.width - 200.0).abs() < 1e-6);
        // CSV: A=83.29070842 cm^2 -> mm^2
        assert!((e.area - 8329.070842).abs() < 1e-3);
        // CSV: Iy=22554.95086 cm^4 -> mm^4
        assert!((e.iy - 225549508.6).abs() < 10.0);
    }

    #[test]
    fn test_pipe_width_falls_back_to_outer_diameter() {
        let list = entries_in(CatalogShape::Pipe, "P-385");
        let e = list
            .iter()
            .find(|e| e.name == "O-400x19")
            .expect("section not found");
        assert!((e.width - e.depth).abs() < 1e-6);
        assert!((e.depth - 400.0).abs() < 1e-6);
    }

    #[test]
    fn test_to_section_carries_name_and_properties() {
        let list = entries_in(CatalogShape::Box, "SHS");
        let e = &list[0];
        let sec = to_section(e, SectionId(7));
        assert_eq!(sec.id, SectionId(7));
        assert_eq!(sec.name, e.name);
        assert_eq!(sec.area, e.area);
        assert_eq!(sec.depth, e.depth);
    }
}
