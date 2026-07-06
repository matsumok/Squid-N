use squid_n_core::model::Model;
use squid_n_edit::UndoStack;
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
    pub results: Box<dyn squid_n_io::results::ResultStore>,
}

#[cfg(feature = "mcp")]
pub mod server {
    use super::*;
    // rmcp 1.x では `tool_router`/`tool_handler` マクロはクレートルート（rmcp_macros の再エクスポート）にあり、
    // `Parameters` は `handler::server::tool` 内で private import されているだけなので
    // 実体を持つ `handler::server::wrapper` から取る必要がある。
    use rmcp::handler::server::tool::ToolRouter;
    use rmcp::handler::server::wrapper::Parameters;
    use rmcp::model::{CallToolResult, Content, Implementation, ServerInfo};
    use rmcp::transport::stdio;
    use rmcp::{tool, tool_handler, tool_router, ErrorData, ServerHandler, ServiceExt};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    #[derive(Clone)]
    pub struct SquidNServer {
        state: Arc<Mutex<ServerState>>,
        /// rmcp の `#[tool_handler]` が生成するディスパッチ用ルータ。
        /// rmcp 1.7 のマクロ展開はこのフィールドを直接読まない経路を取り得るため
        /// dead_code を明示的に許可する（rmcp の標準パターンに従った保持）。
        #[allow(dead_code)]
        tool_router: ToolRouter<Self>,
    }

    impl SquidNServer {
        pub fn new(state: ServerState) -> Self {
            Self {
                state: Arc::new(Mutex::new(state)),
                tool_router: Self::tool_router(),
            }
        }
    }

    #[tool_router]
    impl SquidNServer {
        #[tool(description = "節点・部材・断面を検索する")]
        pub async fn model_query(
            &self,
            Parameters(args): Parameters<QueryArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            // 実クエリは feature 非依存の query_model に委譲（テスト済み）。
            let items = super::query_model(&st.model, &args.kind, args.filter.as_deref());
            let result = QueryResult { items };
            Ok(CallToolResult::success(vec![Content::json(result)?]))
        }

        #[tool(description = "解析を非同期で実行する")]
        pub async fn analysis_run(
            &self,
            Parameters(args): Parameters<AnalysisRunArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            // ジョブ登録・実行中への遷移・モデルの複製はロック保持中に行い、
            // ロックを解放してから spawn_blocking へ渡す
            // （Mutex ガードをスレッドへ持ち込まないため）。
            let (id, model) = {
                let mut st = self.state.lock().await;
                let id = st.jobs.register(args.kind);
                st.jobs.update(&id, JobStatus::Running { progress: 0.0 });
                (id, st.model.clone())
            };

            let state = self.state.clone();
            let job_id = id.clone();
            // 解析（CPU バウンド）は spawn_blocking で実行する。その完了待ちとジョブ状態の
            // 更新は別タスク（tokio::spawn）で行うことで、本ツール呼び出し自体は
            // job_id を即時返し、応答をブロックしない（非同期ジョブとしての仕様どおり）。
            tokio::spawn(async move {
                let outcome =
                    tokio::task::spawn_blocking(move || super::analyze_model(&model)).await;
                let status = match outcome {
                    Ok(Ok(result_json)) => JobStatus::Done {
                        result_ref: result_json,
                    },
                    Ok(Err(e)) => JobStatus::Failed { error: e },
                    // spawn_blocking 内で panic した場合。JoinError を利用者向けメッセージに変換する。
                    Err(join_err) => JobStatus::Failed {
                        error: format!("解析タスクが異常終了しました: {join_err}"),
                    },
                };
                let mut st = state.lock().await;
                st.jobs.update(&job_id, status);
            });

            Ok(CallToolResult::success(vec![Content::json(
                serde_json::json!({ "job_id": id }),
            )?]))
        }

        /// ジョブの現在状態を返す。`status` は `Queued`/`Running{progress}`/
        /// `Done{result_ref}`/`Failed{error}` のいずれか。`Done` の場合
        /// `result_ref` に解析結果（変位ベクトルの JSON 文字列）を保持している。
        #[tool(description = "ジョブの状態を取得する")]
        pub async fn analysis_status(
            &self,
            Parameters(args): Parameters<AnalysisStatusArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            let job = st.jobs.get(&args.job_id);
            match job {
                Some(j) => Ok(CallToolResult::success(vec![Content::json(j)?])),
                // rmcp 1.x では CallToolResult::error は content のみを取り、
                // JSON-RPC レベルのエラーコードは Err(ErrorData) 側で表現する。
                None => Err(ErrorData::invalid_params("job not found", None)),
            }
        }
    }

    #[tool_handler]
    impl ServerHandler for SquidNServer {
        fn get_info(&self) -> ServerInfo {
            // rmcp 1.x では ServerInfo(=InitializeResult)・Implementation は #[non_exhaustive] のため
            // 構造体リテラルで組み立てられない。ビルダーメソッド経由で組み立てる。
            ServerInfo::default().with_server_info(Implementation::new("squid-n-mcp", "0.1.0"))
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
        let service = SquidNServer::new(state).serve(stdio()).await?;
        service.waiting().await?;
        Ok(())
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
            NodalLoad, Node, Section,
        };
        use std::time::Duration;

        /// `ResultStore` のテスト用ダミー実装。
        /// `analysis_run`/`analysis_status` の経路では結果ストアへの読み書きは行わないため、
        /// `manifest` 以外は呼ばれない前提で `unimplemented!` にしている。
        struct NullResultStore {
            manifest: squid_n_io::results::ResultManifest,
        }

        impl squid_n_io::results::ResultStore for NullResultStore {
            fn writer(
                &mut self,
                _case: squid_n_io::results::CaseId,
                _kind: squid_n_io::results::ResultKind,
            ) -> Box<dyn squid_n_io::results::ResultWriter> {
                unimplemented!("テスト用ダミー: analysis_run のテストでは呼ばれない")
            }
            fn query(
                &self,
                _q: &squid_n_io::results::ResultQuery,
            ) -> squid_n_io::results::ResultBatch {
                unimplemented!("テスト用ダミー: analysis_run のテストでは呼ばれない")
            }
            fn manifest(&self) -> &squid_n_io::results::ResultManifest {
                &self.manifest
            }
        }

        fn make_state(model: Model) -> ServerState {
            ServerState {
                model,
                undo: UndoStack::new(),
                jobs: JobRegistry::new(),
                results: Box::new(NullResultStore {
                    manifest: squid_n_io::results::ResultManifest { entries: vec![] },
                }),
            }
        }

        /// 片持ち梁（node0 固定・node1 自由）+ 荷重ケース1つの、解析が完走できる最小モデル。
        fn cantilever_with_load_case() -> Model {
            Model {
                nodes: vec![
                    Node {
                        id: NodeId(0),
                        coord: [0.0, 0.0, 0.0],
                        restraint: Dof6Mask::FIXED,
                        mass: None,
                        story: None,
                    },
                    Node {
                        id: NodeId(1),
                        coord: [1000.0, 0.0, 0.0],
                        restraint: Dof6Mask::FREE,
                        mass: None,
                        story: None,
                    },
                ],
                elements: vec![ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                }],
                sections: vec![Section {
                    id: SectionId(0),
                    name: "beam".into(),
                    area: 100.0,
                    iy: 833.33,
                    iz: 833.33,
                    j: 100.0,
                    depth: 10.0,
                    width: 10.0,
                    as_y: 83.33,
                    as_z: 83.33,
                    panel_thickness: None,
                    thickness: None,
                    shape: None,
                }],
                materials: vec![Material {
                    id: MaterialId(0),
                    name: "mat".into(),
                    young: 20000.0,
                    poisson: 0.3,
                    density: 0.0,
                    shear: None,
                    fc: None,
                    fy: None,
                }],
                load_cases: vec![LoadCase {
                    id: LoadCaseId(1),
                    name: "case1".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                }],
                ..Default::default()
            }
        }

        /// 上と同じモデルから荷重ケースだけを抜いたもの（`analyze_model` が
        /// "no load cases" で失敗する経路を確認するため）。
        fn cantilever_without_load_case() -> Model {
            Model {
                load_cases: vec![],
                ..cantilever_with_load_case()
            }
        }

        /// `CallToolResult`（`analysis_run` の戻り値）から `job_id` を取り出す。
        fn extract_job_id(result: &CallToolResult) -> String {
            let text = result.content[0]
                .raw
                .as_text()
                .expect("analysis_run の応答は text content のはず")
                .text
                .clone();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            value["job_id"].as_str().unwrap().to_string()
        }

        /// ジョブが `Done`/`Failed` のいずれかの終端状態に達するまでポーリングする。
        async fn wait_for_terminal(server: &SquidNServer, job_id: &str) -> JobStatus {
            for _ in 0..200 {
                {
                    let st = server.state.lock().await;
                    if let Some(job) = st.jobs.get(job_id) {
                        match &job.status {
                            JobStatus::Done { .. } | JobStatus::Failed { .. } => {
                                return job.status.clone();
                            }
                            _ => {}
                        }
                    }
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            panic!("job did not reach a terminal state in time");
        }

        #[tokio::test]
        async fn test_analysis_run_completes_for_valid_model() {
            let server = SquidNServer::new(make_state(cantilever_with_load_case()));
            let result = server
                .analysis_run(Parameters(AnalysisRunArgs {
                    kind: JobKind::LinearStatic,
                }))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            assert!(
                matches!(status, JobStatus::Done { .. }),
                "expected Done, got {status:?}"
            );
        }

        #[tokio::test]
        async fn test_analysis_run_fails_without_load_case() {
            let server = SquidNServer::new(make_state(cantilever_without_load_case()));
            let result = server
                .analysis_run(Parameters(AnalysisRunArgs {
                    kind: JobKind::LinearStatic,
                }))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            match status {
                JobStatus::Failed { error } => {
                    assert!(error.contains("no load cases"), "unexpected error: {error}");
                }
                other => panic!("expected Failed, got {other:?}"),
            }
        }
    }
}

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
}

/// `model.query` の中核ロジック（feature 非依存・テスト可能）。
///
/// `kind` で `node`/`member`(=element)/`section` を選び、各要素を JSON 化して返す。
/// `filter` が与えられたときは、各 JSON を文字列化した中に部分一致するものだけを残す
/// （簡易フィルタ。名前・ID 等での絞り込み用）。MCP ツール `model_query` はこれを呼ぶ。
pub fn query_model(model: &Model, kind: &str, filter: Option<&str>) -> Vec<serde_json::Value> {
    use serde_json::json;
    let items: Vec<serde_json::Value> = match kind {
        "node" | "nodes" => model
            .nodes
            .iter()
            .map(|n| {
                json!({
                    "id": n.id.0,
                    "coord": n.coord,
                    "story": n.story.map(|s| s.0),
                })
            })
            .collect(),
        "member" | "members" | "element" | "elements" => model
            .elements
            .iter()
            .map(|e| {
                json!({
                    "id": e.id.0,
                    "kind": format!("{:?}", e.kind),
                    "nodes": e.nodes.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "section": e.section.map(|s| s.0),
                    "material": e.material.map(|m| m.0),
                })
            })
            .collect(),
        "section" | "sections" => model
            .sections
            .iter()
            .map(|s| {
                json!({
                    "id": s.id.0,
                    "name": s.name,
                    "area": s.area,
                    "iy": s.iy,
                    "iz": s.iz,
                })
            })
            .collect(),
        _ => vec![],
    };
    match filter {
        Some(f) if !f.is_empty() => items
            .into_iter()
            .filter(|v| v.to_string().contains(f))
            .collect(),
        _ => items,
    }
}

/// 解析の実処理（feature 非依存・テスト可能）。`Model` の参照だけを受け取るため、
/// `ServerState` のロックを取らずに（= ロック解放後に）呼び出せる。
/// 現状は先頭の荷重ケースに対する線形静的解析のみ（他ジョブ種別は将来対応）。
///
/// `analysis_run`（MCP ツール）は `state.model.clone()` を取ってロックを落としてから
/// `spawn_blocking` 内でこの関数を呼ぶことで、CPU バウンドな解析中も `ServerState` の
/// ミューテックスを他ツール呼び出しのためにブロックしない。
pub fn analyze_model(model: &Model) -> Result<String, String> {
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    if let Some(lc) = model.load_cases.first() {
        let result = analysis
            .linear_static(lc.id)
            .map_err(|e| format!("solve failed: {e}"))?;
        Ok(serde_json::to_string(&result.disp).unwrap_or_default())
    } else {
        Err("no load cases".into())
    }
}

/// `analyze_model` の `ServerState` 経由の薄いラッパ（後方互換用）。
pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    analyze_model(&state.model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, LocalAxis, Node, Section};

    fn sample_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: squid_n_core::dof::Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: squid_n_core::dof::Dof6Mask::FREE,
                    mass: None,
                    story: Some(squid_n_core::ids::StoryId(0)),
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "H-400".to_string(),
                area: 100.0,
                iy: 1000.0,
                iz: 2000.0,
                j: 50.0,
                depth: 400.0,
                width: 200.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [
                    squid_n_core::model::EndCondition::Fixed,
                    squid_n_core::model::EndCondition::Fixed,
                ],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_query_model_nodes() {
        let m = sample_model();
        let items = query_model(&m, "node", None);
        assert_eq!(items.len(), 2);
        assert_eq!(items[0]["id"], 0);
        assert_eq!(items[1]["story"], 0);
    }

    #[test]
    fn test_query_model_elements_and_sections() {
        let m = sample_model();
        assert_eq!(query_model(&m, "member", None).len(), 1);
        let secs = query_model(&m, "section", None);
        assert_eq!(secs.len(), 1);
        assert_eq!(secs[0]["name"], "H-400");
    }

    #[test]
    fn test_query_model_filter() {
        let m = sample_model();
        // 名前で絞り込み（断面名 H-400 を含むものだけ）。
        assert_eq!(query_model(&m, "section", Some("H-400")).len(), 1);
        assert_eq!(query_model(&m, "section", Some("RC")).len(), 0);
    }

    #[test]
    fn test_query_model_unknown_kind() {
        let m = sample_model();
        assert!(query_model(&m, "bogus", None).is_empty());
    }

    #[test]
    fn test_job_registry_lifecycle() {
        let mut reg = JobRegistry::new();
        let id = reg.register(JobKind::LinearStatic);
        assert!(matches!(reg.get(&id).unwrap().status, JobStatus::Queued));
        reg.update(&id, JobStatus::Running { progress: 0.5 });
        assert!(matches!(
            reg.get(&id).unwrap().status,
            JobStatus::Running { progress } if (progress - 0.5).abs() < 1e-6
        ));
        reg.update(
            &id,
            JobStatus::Done {
                result_ref: "r1".into(),
            },
        );
        assert!(matches!(
            &reg.get(&id).unwrap().status,
            JobStatus::Done { result_ref } if result_ref == "r1"
        ));
        // 異なる ID は別ジョブ。
        let id2 = reg.register(JobKind::Eigen);
        assert_ne!(id, id2);
        assert!(reg.get("nonexistent").is_none());
    }
}
