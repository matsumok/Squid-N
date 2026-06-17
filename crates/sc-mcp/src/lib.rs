use sc_core::model::Model;
use sc_edit::UndoStack;
use std::collections::HashMap;

pub type JobId = String;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, schemars::JsonSchema)]
pub enum JobStatus {
    Queued,
    Running { progress: f32 },
    Done { result_ref: String },
    Failed { error: String },
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, schemars::JsonSchema,
)]
pub enum JobKind {
    LinearStatic,
    Eigen,
    Pushover,
    TimeHistory,
    DesignCheck,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Job {
    pub id: JobId,
    pub kind: JobKind,
    pub status: JobStatus,
}

#[derive(Debug, Default)]
pub struct JobRegistry {
    jobs: HashMap<JobId, Job>,
    next_id: u64,
}

impl JobRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, kind: JobKind) -> JobId {
        let id = format!("job-{}", self.next_id);
        self.next_id += 1;
        let job = Job {
            id: id.clone(),
            kind,
            status: JobStatus::Queued,
        };
        self.jobs.insert(id.clone(), job);
        id
    }

    pub fn get(&self, id: &str) -> Option<&Job> {
        self.jobs.get(id)
    }

    pub fn update(&mut self, id: &str, status: JobStatus) {
        if let Some(job) = self.jobs.get_mut(id) {
            job.status = status;
        }
    }
}

pub struct ServerState {
    pub model: Model,
    pub undo: UndoStack,
    pub jobs: JobRegistry,
    pub results: Box<dyn sc_io::results::ResultStore>,
}

#[cfg(feature = "mcp")]
pub mod server {
    use super::*;
    use rmcp::handler::server::router::tool::ToolRouter as _;
    use rmcp::handler::server::tool::{tool_router, Parameters, ToolRouter};
    use rmcp::model::{CallToolResult, Content, ServerInfo};
    use rmcp::transport::stdio;
    use rmcp::{tool, tool_handler, ErrorData, ServerHandler, ServiceExt};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    #[derive(Clone)]
    pub struct StructCalcServer {
        state: Arc<Mutex<ServerState>>,
        tool_router: ToolRouter<Self>,
    }

    impl StructCalcServer {
        pub fn new(state: ServerState) -> Self {
            Self {
                state: Arc::new(Mutex::new(state)),
                tool_router: Self::tool_router(),
            }
        }
    }

    #[tool_router]
    impl StructCalcServer {
        #[tool(description = "節点・部材・断面を検索する")]
        pub async fn model_query(
            &self,
            Parameters(args): Parameters<QueryArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            let _ = &st.model;
            let result = QueryResult { items: vec![] };
            Ok(CallToolResult::success(vec![Content::json(result)?]))
        }

        #[tool(description = "解析を非同期で実行する")]
        pub async fn analysis_run(
            &self,
            Parameters(args): Parameters<AnalysisRunArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let mut st = self.state.lock().await;
            let id = st.jobs.register(args.kind);
            Ok(CallToolResult::success(vec![Content::json(
                serde_json::json!({ "job_id": id }),
            )?]))
        }

        #[tool(description = "ジョブの状態を取得する")]
        pub async fn analysis_status(
            &self,
            Parameters(args): Parameters<AnalysisStatusArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            let job = st.jobs.get(&args.job_id);
            match job {
                Some(j) => Ok(CallToolResult::success(vec![Content::json(j)?])),
                None => Ok(CallToolResult::error(
                    rmcp::model::ErrorCode::InvalidParams,
                    "job not found",
                )),
            }
        }
    }

    #[tool_handler]
    impl ServerHandler for StructCalcServer {
        fn get_info(&self) -> ServerInfo {
            ServerInfo {
                name: "sc-mcp".into(),
                version: "0.1.0".into(),
                ..Default::default()
            }
        }
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct QueryArgs {
        pub kind: String,
        pub filter: Option<String>,
    }

    #[derive(Debug, serde::Serialize, schemars::JsonSchema)]
    pub struct QueryResult {
        pub items: Vec<serde_json::Value>,
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct AnalysisRunArgs {
        pub kind: JobKind,
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct AnalysisStatusArgs {
        pub job_id: String,
    }

    pub async fn run_stdio_server(state: ServerState) -> Result<(), Box<dyn std::error::Error>> {
        let service = StructCalcServer::new(state).serve(stdio()).await?;
        service.waiting().await?;
        Ok(())
    }
}

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
}

pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    let analysis = sc_solver::analysis::Analysis::prepare(&state.model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    if let Some(lc) = state.model.load_cases.first() {
        let result = analysis
            .linear_static(lc.id)
            .map_err(|e| format!("solve failed: {e}"))?;
        Ok(serde_json::to_string(&result.disp).unwrap_or_default())
    } else {
        Err("no load cases".into())
    }
}
