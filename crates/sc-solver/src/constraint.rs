use faer::sparse::SparseColMat;
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::model::{Constraint, Model};
use sc_math::sparse::{assemble_csc, Triplet};

pub struct Reducer {
    pub t_rows: Vec<Vec<(usize, f64)>>,
    pub n_indep: usize,
    pub n_free: usize,
}

impl Reducer {
    pub fn build(model: &Model, dofmap: &DofMap) -> Self {
        let n_free = dofmap.n_active();
        let mut t_rows: Vec<Vec<(usize, f64)>> = (0..n_free).map(|i| vec![(i, 1.0)]).collect();
        let node_coords: Vec<[f64; 3]> = model.nodes.iter().map(|n| n.coord).collect();

        for constraint in &model.constraints {
            if let Constraint::Mpc { master, terms } = constraint {
                let mi = master.index();
                for &(slave_node, slave_dof, coef) in terms {
                    let si = slave_node.index();
                    let sg = si * DOF_PER_NODE + slave_dof as usize;
                    if let Some(s_active) = dofmap.active(sg) {
                        let s_idx = s_active as usize;
                        let mg = mi * DOF_PER_NODE + slave_dof as usize;
                        if let Some(m_active) = dofmap.active(mg) {
                            if s_idx < t_rows.len() {
                                t_rows[s_idx] = vec![(m_active as usize, coef)];
                            }
                        }
                    }
                }
            }
        }
        for constraint in &model.constraints {
            match constraint {
                Constraint::RigidDiaphragm {
                    story: _,
                    master,
                    slaves,
                } => {
                    let mi = master.index();
                    let mx = node_coords[mi][0];
                    let my = node_coords[mi][1];
                    for &slave in slaves {
                        let si = slave.index();
                        let sx = node_coords[si][0];
                        let sy = node_coords[si][1];
                        let dx = sx - mx;
                        let dy = sy - my;
                        // Ux
                        let sg_ux = si * DOF_PER_NODE + 0;
                        let mg_ux = mi * DOF_PER_NODE + 0;
                        let mg_rz = mi * DOF_PER_NODE + 5;
                        if let Some(sa) = dofmap.active(sg_ux) {
                            let s_idx = sa as usize;
                            let mut row = Vec::new();
                            if let Some(ma) = dofmap.active(mg_ux) {
                                row.push((ma as usize, 1.0));
                            }
                            if let Some(ma) = dofmap.active(mg_rz) {
                                row.push((ma as usize, -dy));
                            }
                            if s_idx < t_rows.len() {
                                t_rows[s_idx] = row;
                            }
                        }
                        // Uy
                        let sg_uy = si * DOF_PER_NODE + 1;
                        let mg_uy = mi * DOF_PER_NODE + 1;
                        if let Some(sa) = dofmap.active(sg_uy) {
                            let s_idx = sa as usize;
                            let mut row = Vec::new();
                            if let Some(ma) = dofmap.active(mg_uy) {
                                row.push((ma as usize, 1.0));
                            }
                            if let Some(ma) = dofmap.active(mg_rz) {
                                row.push((ma as usize, dx));
                            }
                            if s_idx < t_rows.len() {
                                t_rows[s_idx] = row;
                            }
                        }
                        // Rz
                        let sg_rz = si * DOF_PER_NODE + 5;
                        if let Some(sa) = dofmap.active(sg_rz) {
                            let s_idx = sa as usize;
                            if let Some(ma) = dofmap.active(mg_rz) {
                                if s_idx < t_rows.len() {
                                    t_rows[s_idx] = vec![(ma as usize, 1.0)];
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        let mut is_indep = vec![false; t_rows.len()];
        for (i, row) in t_rows.iter().enumerate() {
            if row.len() == 1 && row[0].0 == i && (row[0].1 - 1.0).abs() < 1e-12 {
                is_indep[i] = true;
            }
        }

        let mut new_indep = vec![usize::MAX; t_rows.len()];
        let mut counter = 0usize;
        for i in 0..t_rows.len() {
            if is_indep[i] {
                new_indep[i] = counter;
                counter += 1;
            }
        }
        for row in &t_rows {
            for &(idx, _) in row {
                if idx < new_indep.len() && new_indep[idx] == usize::MAX {
                    new_indep[idx] = counter;
                    counter += 1;
                }
            }
        }

        let remapped: Vec<Vec<(usize, f64)>> = t_rows
            .into_iter()
            .map(|row| {
                row.into_iter()
                    .filter_map(|(idx, val)| {
                        if idx < new_indep.len() && new_indep[idx] != usize::MAX {
                            Some((new_indep[idx], val))
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .collect();

        Reducer {
            t_rows: remapped,
            n_indep: counter,
            n_free,
        }
    }

    pub fn reduce_k(&self, k_free: &SparseColMat<usize, f64>) -> SparseColMat<usize, f64> {
        let mut triplets = Vec::new();
        for i in 0..self.n_free {
            let ti_list = &self.t_rows[i];
            if ti_list.is_empty() {
                continue;
            }
            for j in 0..self.n_free {
                let tj_list = &self.t_rows[j];
                if tj_list.is_empty() {
                    continue;
                }
                let v_entry = k_free.get(i, j);
                let v = match v_entry {
                    Some(&val) => val,
                    None => 0.0,
                };
                if v == 0.0 {
                    continue;
                }
                for &(a, ta) in ti_list {
                    for &(b, tb) in tj_list {
                        triplets.push(Triplet {
                            row: a,
                            col: b,
                            val: ta * v * tb,
                        });
                    }
                }
            }
        }
        assemble_csc(self.n_indep, triplets)
    }

    pub fn reduce_f(&self, f_free: &[f64]) -> Vec<f64> {
        let mut f_red = vec![0.0; self.n_indep];
        for i in 0..self.n_free {
            if f_free[i] != 0.0 {
                for &(a, ta) in &self.t_rows[i] {
                    f_red[a] += ta * f_free[i];
                }
            }
        }
        f_red
    }

    pub fn expand_u(&self, u_indep: &[f64]) -> Vec<f64> {
        let mut u_free = vec![0.0; self.n_free];
        for i in 0..self.n_free {
            for &(a, ta) in &self.t_rows[i] {
                u_free[i] += ta * u_indep[a];
            }
        }
        u_free
    }
}
