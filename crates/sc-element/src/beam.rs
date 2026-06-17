use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::{ElemId, NodeId};
use sc_core::model::{EndCondition, Material, Model, Section};
use smallvec::SmallVec;

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub enum ZoneSource {
    Auto,
    Manual,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct RigidZone {
    pub length_i: f64,
    pub length_j: f64,
    pub source_i: ZoneSource,
    pub source_j: ZoneSource,
    pub reduction: f64,
}

impl Default for RigidZone {
    fn default() -> Self {
        Self {
            length_i: 0.0,
            length_j: 0.0,
            source_i: ZoneSource::Auto,
            source_j: ZoneSource::Auto,
            reduction: 1.0,
        }
    }
}

pub struct RigidZoneRule {
    pub reduction: f64,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self { reduction: 1.0 }
    }
}

#[derive(Clone, Debug)]
pub struct MemberForces {
    pub at: Vec<(f64, [f64; 6])>,
}

#[derive(Clone)]
pub struct BeamElement {
    pub id: ElemId,
    pub e: f64,
    pub g: f64,
    pub a: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
    pub length: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub rigid: RigidZone,
    pub end_cond: [EndCondition; 2],
    pub eval_sections: Vec<f64>,
    pub section: Option<sc_core::ids::SectionId>,
    pub material: Option<sc_core::ids::MaterialId>,
}

fn get_section(model: &Model, sid: Option<sc_core::ids::SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section {
        id: sc_core::ids::SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
    })
}

fn get_material(model: &Model, mid: Option<sc_core::ids::MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        id: sc_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
    })
}

impl BeamElement {
    pub fn new(data: &sc_core::model::ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = if n0.index() < model.nodes.len() {
            model.nodes[n0.index()].coord
        } else {
            [0.0; 3]
        };
        let p1 = if n1.index() < model.nodes.len() {
            model.nodes[n1.index()].coord
        } else {
            [0.0; 3]
        };
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, data.material);
        let g = mat.shear_modulus();

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            sc_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            sc_core::model::rect_shear_area(sec.area)
        };

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: sec.area,
            iy: sec.iy,
            iz: sec.iz,
            j: sec.j,
            as_y,
            as_z,
            length: len,
            nodes: [n0, n1],
            axis,
            rigid: RigidZone::default(),
            end_cond: data.end_cond,
            eval_sections: vec![0.0, 0.5, 1.0],
            section: data.section,
            material: data.material,
        }
    }

    pub fn local_stiffness_raw(&self) -> LocalMat {
        let (e, g, a, iy, iz, jj, l) = (
            self.e,
            self.g,
            self.a,
            self.iy,
            self.iz,
            self.j,
            self.length,
        );
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let phiz = 12.0 * e * iz / (g * self.as_y * l * l);
        let phiy = 12.0 * e * iy / (g * self.as_z * l * l);
        let az = e * iz / ((1.0 + phiz) * l * l * l);
        let ay = e * iy / ((1.0 + phiy) * l * l * l);

        let mut k = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            k.set(i, j, v);
            if i != j {
                k.set(j, i, v);
            }
        };

        s(0, 0, e * a / l);
        s(6, 6, e * a / l);
        s(0, 6, -e * a / l);
        s(3, 3, g * jj / l);
        s(9, 9, g * jj / l);
        s(3, 9, -g * jj / l);

        s(1, 1, 12.0 * az);
        s(7, 7, 12.0 * az);
        s(1, 7, -12.0 * az);
        s(1, 5, 6.0 * az * l);
        s(1, 11, 6.0 * az * l);
        s(5, 7, -6.0 * az * l);
        s(7, 11, -6.0 * az * l);
        s(5, 5, (4.0 + phiz) * az * l * l);
        s(11, 11, (4.0 + phiz) * az * l * l);
        s(5, 11, (2.0 - phiz) * az * l * l);

        s(2, 2, 12.0 * ay);
        s(8, 8, 12.0 * ay);
        s(2, 8, -12.0 * ay);
        s(2, 4, -6.0 * ay * l);
        s(2, 10, -6.0 * ay * l);
        s(4, 8, 6.0 * ay * l);
        s(8, 10, 6.0 * ay * l);
        s(4, 4, (4.0 + phiy) * ay * l * l);
        s(10, 10, (4.0 + phiy) * ay * l * l);
        s(4, 10, (2.0 - phiy) * ay * l * l);

        k
    }

    fn apply_rigid_zone_transform(&self, k_flex: &LocalMat, li: f64, lj: f64) -> LocalMat {
        if li.abs() < 1e-12 && lj.abs() < 1e-12 {
            return LocalMat {
                n: k_flex.n,
                data: k_flex.data.clone(),
            };
        }
        // Tr: 12×12 — flex端自由度(i', j') → 節点自由度(i, j)
        // i' = i を li だけずらし, j' = j を lj だけずらす
        // Tr はほとんど単位行列。i端: ux_i'=ux_i, uy_i'=uy_i-li*rz_i, uz_i'=uz_i+li*ry_i,
        //   rx_i'=rx_i, ry_i'=ry_i, rz_i'=rz_i
        // j端: ux_j'=ux_j, uy_j'=uy_j+lj*rz_j, uz_j'=uz_j-lj*ry_j,
        //   rx_j'=rx_j, ry_j'=ry_j, rz_j'=rz_j
        let mut tr = LocalMat::zeros(12);
        for i in 0..12 {
            tr.set(i, i, 1.0);
        }
        // i端 (index 0..5): uy方向(1) ← rz方向(5) の項
        tr.set(1, 5, -li);
        tr.set(2, 4, li);
        // j端 (index 6..11): uy方向(7) ← rz方向(11) の項
        tr.set(7, 11, lj);
        tr.set(8, 10, -lj);

        // K_node = Tr^T * K_flex * Tr
        let mut tmp = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += k_flex.get(i, k) * tr.get(k, j);
                }
                tmp.set(i, j, s);
            }
        }
        let mut kn = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += tr.get(k, i) * tmp.get(k, j);
                }
                kn.set(i, j, s);
            }
        }
        kn
    }

    #[allow(dead_code)]
    fn condense_end_springs(&self, k_in: &LocalMat) -> LocalMat {
        let mut k = LocalMat {
            n: k_in.n,
            data: k_in.data.clone(),
        };
        // Pinned: free rx at that end → set relevant rows/cols to slave
        // For simplicity, handle Fixed and Pinned via penalty/condensation.
        // Pinned at i-end: Mz_i=0, My_i=0 → remove dofs 4,5 and condense
        // Pinned at j-end: Mz_j=0, My_j=0 → remove dofs 10,11 and condense
        // SemiRigid: add spring stiffness to diagonal and condense out internal dofs

        let mut to_condense = Vec::new();
        match self.end_cond[0] {
            EndCondition::Fixed => {}
            EndCondition::Pinned => {
                to_condense.push(4);
                to_condense.push(5);
            }
            EndCondition::SemiRigid { k_theta } => {
                // Add spring at rotation dofs
                k.set(4, 4, k.get(4, 4) + k_theta);
                k.set(5, 5, k.get(5, 5) + k_theta);
                to_condense.push(4);
                to_condense.push(5);
            }
        }
        match self.end_cond[1] {
            EndCondition::Fixed => {}
            EndCondition::Pinned => {
                to_condense.push(10);
                to_condense.push(11);
            }
            EndCondition::SemiRigid { k_theta } => {
                k.set(10, 10, k.get(10, 10) + k_theta);
                k.set(11, 11, k.get(11, 11) + k_theta);
                to_condense.push(10);
                to_condense.push(11);
            }
        }

        if to_condense.is_empty() {
            return k;
        }

        // Static condensation: partition into keep (a) and remove (b)
        let n = 12;
        let keep: Vec<usize> = (0..n).filter(|i| !to_condense.contains(i)).collect();
        let na = keep.len();
        let nb = to_condense.len();

        // Build Kaa, Kab, Kba, Kbb from k
        let idx = |r: usize, c: usize, n: usize| r * n + c;
        let mut kaa = vec![0.0; na * na];
        let mut kab = vec![0.0; na * nb];
        let mut kba = vec![0.0; nb * na];
        let mut kbb = vec![0.0; nb * nb];

        for (ai, &i) in keep.iter().enumerate() {
            for (aj, &j) in keep.iter().enumerate() {
                kaa[idx(ai, aj, na)] = k.get(i, j);
            }
            for (bj, &j) in to_condense.iter().enumerate() {
                kab[idx(ai, bj, na)] = k.get(i, j);
            }
        }
        for (bi, &i) in to_condense.iter().enumerate() {
            for (aj, &j) in keep.iter().enumerate() {
                kba[idx(bi, aj, nb)] = k.get(i, j);
            }
            for (bj, &j) in to_condense.iter().enumerate() {
                kbb[idx(bi, bj, nb)] = k.get(i, j);
            }
        }

        // Invert Kbb (small, 2-4 dofs max) via Gaussian elimination
        let kbb_inv = invert_small(&kbb, nb);

        // K* = Kaa - Kab * Kbb^{-1} * Kba
        let mut kab_kbbinv = vec![0.0; na * nb];
        for i in 0..na {
            for j in 0..nb {
                let mut s = 0.0;
                for k in 0..nb {
                    s += kab[idx(i, k, na)] * kbb_inv[idx(k, j, nb)];
                }
                kab_kbbinv[idx(i, j, na)] = s;
            }
        }

        let mut kstar = LocalMat::zeros(na);
        for i in 0..na {
            for j in 0..na {
                let mut s = kaa[idx(i, j, na)];
                for k in 0..nb {
                    s -= kab_kbbinv[idx(i, k, na)] * kba[idx(k, j, nb)];
                }
                kstar.set(i, j, s);
            }
        }
        kstar
    }

    pub fn local_stiffness(&self) -> LocalMat {
        let l_flex = self.length - self.rigid.length_i - self.rigid.length_j;
        let k_raw = if l_flex > 1e-12 {
            let mut beam = BeamElement {
                length: l_flex,
                ..BeamElement {
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    ..self.clone()
                }
            };
            // Temporarily use Fixed ends for raw stiffness
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
            beam.local_stiffness_raw()
        } else {
            LocalMat::zeros(12)
        };

        let li = self.rigid.length_i;
        let lj = self.rigid.length_j;
        self.apply_rigid_zone_transform(&k_raw, li, lj)
    }

    pub fn recover_forces(&self, u_elem_global: &[f64; 12]) -> MemberForces {
        let u_local = self.axis.rotate_to_local(u_elem_global);
        let k_local = self.local_stiffness_raw();
        // f_local = K_local * u_local (in local coords)
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        // N, Qy, Qz, Mx, My, Mz at i-end: f_local[0], f_local[1], f_local[2], f_local[3], f_local[4], f_local[5]
        // j-end: f_local[6], f_local[7], f_local[8], f_local[9], f_local[10], f_local[11]

        let mut at = Vec::new();
        for &xi in &self.eval_sections {
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = f_local[0] * (1.0 - xi) + f_local[6] * xi;
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = f_local[3];
                let my = f_local[4] - f_local[2] * xi * self.length;
                let mz = f_local[5] + f_local[1] * xi * self.length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[0] * (1.0 - xi) + f_local[6] * xi;
                let qy = -f_local[7];
                let qz = -f_local[8];
                let mx = f_local[9];
                let my = f_local[10] - f_local[8] * (1.0 - xi) * self.length;
                let mz = f_local[11] + f_local[7] * (1.0 - xi) * self.length;
                (n, qy, qz, mx, my, mz)
            };
            at.push((xi, [n, qy, qz, mx, my, mz]));
        }

        MemberForces { at }
    }
}

#[allow(dead_code)]
fn invert_small(a: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * n * 2];
    for i in 0..n {
        for j in 0..n {
            aug[i * (2 * n) + j] = a[i * n + j];
        }
        aug[i * (2 * n) + n + i] = 1.0;
    }
    for col in 0..n {
        let mut pivot = aug[col * (2 * n) + col];
        if pivot.abs() < 1e-15 {
            pivot = 1.0;
        }
        for j in 0..2 * n {
            aug[col * (2 * n) + j] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row * (2 * n) + col];
                for j in 0..2 * n {
                    aug[row * (2 * n) + j] -= factor * aug[col * (2 * n) + j];
                }
            }
        }
    }
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * (2 * n) + n + j];
        }
    }
    inv
}

pub fn auto_rigid_zones(
    model: &sc_core::model::Model,
    elem_id: sc_core::ids::ElemId,
    rule: &RigidZoneRule,
) -> RigidZone {
    let _ = model;
    let _ = elem_id;
    RigidZone {
        reduction: rule.reduction,
        ..Default::default()
    }
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
    }
}

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        self.local_stiffness()
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_beam() -> BeamElement {
        BeamElement {
            id: ElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 80000.0,
            iy: 1.0666667e9,
            iz: 1.0666667e9,
            j: 0.0,
            as_y: 66666.67,
            as_z: 66666.67,
            length: 3000.0,
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame {
                rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            rigid: RigidZone::default(),
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: None,
            material: None,
        }
    }

    #[test]
    fn test_local_stiffness_symmetric() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                    "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                    k.get(i, j),
                    k.get(j, i)
                );
            }
        }
    }

    #[test]
    fn test_phi_zero_converges_to_bernoulli() {
        // As → ∞ => phi → 0 => Timoshenko → Bernoulli
        let mut beam = make_test_beam();
        beam.as_y = 1e30;
        beam.as_z = 1e30;
        let k_timo = beam.local_stiffness_raw();

        // Bernoulli reference: same beam with phi=0
        let e = beam.e;
        let iz = beam.iz;
        let iy = beam.iy;
        let a = beam.a;
        let l = beam.length;
        let g = beam.g;
        let jj = beam.j;

        let az = e * iz / (l * l * l);
        let ay = e * iy / (l * l * l);

        for i in 0..12 {
            for j in 0..12 {
                let norm_pair = if i <= j { (i, j) } else { (j, i) };
                let bernoulli = match norm_pair {
                    (0, 0) | (6, 6) => e * a / l,
                    (0, 6) => -e * a / l,
                    (3, 3) | (9, 9) => g * jj / l,
                    (3, 9) => -g * jj / l,
                    (1, 1) | (7, 7) => 12.0 * az,
                    (1, 7) => -12.0 * az,
                    (1, 5) | (1, 11) => 6.0 * az * l,
                    (5, 7) | (7, 11) => -6.0 * az * l,
                    (5, 5) | (11, 11) => 4.0 * az * l * l,
                    (5, 11) => 2.0 * az * l * l,
                    (2, 2) | (8, 8) => 12.0 * ay,
                    (2, 8) => -12.0 * ay,
                    (2, 4) | (2, 10) => -6.0 * ay * l,
                    (4, 8) | (8, 10) => 6.0 * ay * l,
                    (4, 4) | (10, 10) => 4.0 * ay * l * l,
                    (4, 10) => 2.0 * ay * l * l,
                    _ => 0.0,
                };
                let timo = k_timo.get(i, j);
                assert!(
                    (timo - bernoulli).abs() < 1e-6,
                    "K[{i}][{j}]: timo={timo}, bernoulli={bernoulli}"
                );
            }
        }
    }

    #[test]
    fn test_beam_axial_stiffness() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        let ea_l = beam.e * beam.a / beam.length;
        assert!((k.get(0, 0) - ea_l).abs() < 1e-9);
        assert!((k.get(0, 6) + ea_l).abs() < 1e-9);
        assert!((k.get(6, 0) + ea_l).abs() < 1e-9);
        assert!((k.get(6, 6) - ea_l).abs() < 1e-9);
    }

    #[test]
    fn test_beam_torsion_stiffness() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        let gj_l = beam.g * beam.j / beam.length;
        assert!((k.get(3, 3) - gj_l).abs() < 1e-9);
        assert!((k.get(9, 9) - gj_l).abs() < 1e-9);
        assert!((k.get(3, 9) + gj_l).abs() < 1e-9);
    }
}
