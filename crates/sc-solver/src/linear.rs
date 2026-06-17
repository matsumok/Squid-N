use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use sc_core::dof::DofMap;
use sc_core::ids::LoadCaseId;
use sc_core::model::Model;
use sc_element::beam::MemberForces;
use sc_element::factory::build_behavior;
use sc_math::solver::{make_solver, SolveError, SolverBackend};

pub struct StaticOnce {
    pub disp: Vec<[f64; 6]>,
    pub member_forces: Vec<(sc_core::ids::ElemId, MemberForces)>,
}

pub fn linear_static_once(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        });
    }

    let k_free = assemble_global_k(model, &dofmap);
    let f_free = assemble_global_f(model, &dofmap, lc);

    let reducer = Reducer::build(model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let f_red = reducer.reduce_f(&f_free);
    let n_indep = reducer.n_indep;

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    if n_indep > 0 {
        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; model.nodes.len()];
        for ni in 0..model.nodes.len() {
            for d in 0..sc_core::dof::DOF_PER_NODE {
                let g = ni * sc_core::dof::DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    let val = u_free[active as usize];
                    match d {
                        0 => disp[ni][0] = val,
                        1 => disp[ni][1] = val,
                        2 => disp[ni][2] = val,
                        3 => disp[ni][3] = val,
                        4 => disp[ni][4] = val,
                        _ => disp[ni][5] = val,
                    }
                }
            }
        }

        // Recover forces for each element
        let mut member_forces = Vec::new();
        let _ctx = sc_element::behavior::Ctx { model };
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let mut u_elem = [0.0_f64; 12];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() && k < 12 {
                    u_elem[k] = u_free[g];
                }
            }

            if let Some(forces) = behavior.recover_forces(&u_elem) {
                member_forces.push((elem.id, forces));
            }
        }

        Ok(StaticOnce {
            disp,
            member_forces,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        })
    }
}
