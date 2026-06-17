pub struct NewmarkCfg {
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
}

impl NewmarkCfg {
    /// 平均加速度法（無条件安定）。dt は後で設定する。
    pub fn average_accel() -> Self {
        Self {
            beta: 0.25,
            gamma: 0.5,
            dt: 0.0,
        }
    }
    /// 線形加速度法（条件付安定）。dt は後で設定する。
    pub fn linear_accel() -> Self {
        Self {
            beta: 1.0 / 6.0,
            gamma: 0.5,
            dt: 0.0,
        }
    }
}

pub struct HhtCfg {
    pub alpha: f64,
    pub dt: f64,
}

impl HhtCfg {
    pub fn new(dt: f64) -> Self {
        Self { alpha: -0.1, dt }
    }
}

pub struct GroundMotion {
    pub dt: f64,
    pub accel_x: Vec<f64>,
    pub accel_y: Option<Vec<f64>>,
}

/// 時刻歴応答解析の結果（設計書 §10.5）。
/// 時系列の全量は結果I/O（§6）へストリーミングし、メモリに全保持しない。
pub struct ResponseResult {
    pub time: Vec<f64>,
    pub peak_disp: Vec<[f64; 6]>,
    pub story_drift_angle: Vec<f64>,
    pub cumulative_ductility: Vec<f64>,
}
