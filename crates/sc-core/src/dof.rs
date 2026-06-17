use crate::model::Model;

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum Dof {
    Ux = 0,
    Uy = 1,
    Uz = 2,
    Rx = 3,
    Ry = 4,
    Rz = 5,
}

pub const DOF_PER_NODE: usize = 6;

#[derive(Clone, Copy, Default, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub struct Dof6Mask(pub u8);

impl Dof6Mask {
    pub const FREE: Self = Dof6Mask(0b000000);
    pub const FIXED: Self = Dof6Mask(0b111111);
    pub const PINNED: Self = Dof6Mask(0b000111);
    pub fn is_fixed(self, d: Dof) -> bool {
        self.0 & (1 << d as u8) != 0
    }
    pub fn set_fixed(&mut self, d: Dof) {
        self.0 |= 1 << d as u8;
    }
}

pub type GlobalDof = usize;

#[derive(Clone, Debug, Default)]
pub struct DofMap {
    active_of: Vec<Option<u32>>,
    global_of: Vec<GlobalDof>,
    n_active: usize,
}

impl DofMap {
    pub fn build(model: &Model) -> Self {
        let n_global = model.nodes.len() * DOF_PER_NODE;
        let mut active_of = vec![None; n_global];
        let mut global_of = Vec::new();
        let mut counter = 0u32;
        for (ni, node) in model.nodes.iter().enumerate() {
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                let dof = match d {
                    0 => Dof::Ux,
                    1 => Dof::Uy,
                    2 => Dof::Uz,
                    3 => Dof::Rx,
                    4 => Dof::Ry,
                    _ => Dof::Rz,
                };
                if !node.restraint.is_fixed(dof) {
                    active_of[g] = Some(counter);
                    global_of.push(g);
                    counter += 1;
                }
            }
        }
        DofMap {
            active_of,
            global_of,
            n_active: counter as usize,
        }
    }

    pub fn n_active(&self) -> usize {
        self.n_active
    }
    pub fn active(&self, g: GlobalDof) -> Option<u32> {
        self.active_of[g]
    }
    pub fn global(&self, a: u32) -> GlobalDof {
        self.global_of[a as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dof::Dof6Mask;
    use crate::ids::*;
    use crate::model::*;

    fn make_model_with_restraints(restraints: &[Dof6Mask]) -> Model {
        let nodes: Vec<Node> = restraints
            .iter()
            .enumerate()
            .map(|(i, &r)| Node {
                id: NodeId(i as u32),
                coord: [0.0; 3],
                restraint: r,
                mass: None,
                story: None,
            })
            .collect();
        Model {
            nodes,
            ..Default::default()
        }
    }

    #[test]
    fn test_all_free() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE; 3]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 18);
    }

    #[test]
    fn test_one_fixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE, Dof6Mask::FIXED, Dof6Mask::FREE]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 12);
    }

    #[test]
    fn test_all_fixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FIXED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 0);
    }

    #[test]
    fn test_pinned() {
        let model = make_model_with_restraints(&[Dof6Mask::PINNED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 3);
    }

    #[test]
    fn test_mixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE, Dof6Mask::PINNED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 6 + 3);
    }
}
