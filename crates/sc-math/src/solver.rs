use faer::sparse::SparseColMat;

pub trait LinearSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError>;
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError>;
}

#[derive(Debug, thiserror::Error)]
pub enum SolveError {
    #[error("not factorized yet")]
    NotFactorized,
    #[error("matrix not positive definite")]
    NotPositiveDefinite,
    #[error("dimension mismatch: K={k}, rhs={rhs}")]
    DimMismatch { k: usize, rhs: usize },
    #[error("faer error: {0}")]
    Backend(String),
}

#[derive(Clone, Copy, Debug)]
pub enum SolverBackend {
    DirectSparseCholesky,
    DirectSparseLu,
    IterativePcg { tol: f64, max_iter: usize },
}

pub fn make_solver(backend: SolverBackend) -> Box<dyn LinearSolver> {
    match backend {
        SolverBackend::DirectSparseCholesky => Box::new(crate::cholesky::CholeskySolver::default()),
        _ => unimplemented!("P0 supports DirectSparseCholesky only"),
    }
}
