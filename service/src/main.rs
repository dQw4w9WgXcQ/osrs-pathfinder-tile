use axum::{
    extract::State,
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{get, post},
    Json,
    Router,
};
use catch_panic::CatchPanicLayer;
use derive_new::new;
use log::info;
use osrs_pathfinder_tile::{minify_path, Algo, Point, TilePathfinder};
use serde::{Deserialize, Serialize};
use tower_http::catch_panic;

#[derive(Clone)]
struct AppState {
    tile_pathfinder: &'static TilePathfinder,
}

impl AppState {
    fn new(tile_pathfinder: TilePathfinder) -> Self {
        Self {
            tile_pathfinder: Box::leak(Box::new(tile_pathfinder)),
        }
    }
}

#[tokio::main]
async fn main() {
    println!("================Starting server================");
    tracing_subscriber::fmt::init();

    let port = std::env::var("PORT").unwrap_or("8081".to_string());
    info!("port {}", port);

    let tile_pathfinder = TilePathfinder::load("./grid.zip").unwrap();
    info!("loaded grid.zip");

    let app_state = AppState::new(tile_pathfinder);

    let app = Router::new()
        .route("/", get(|| async { "Hello, World!" }))
        .route("/find-path", post(find_path))
        .route("/find-distances", post(find_distances))
        .layer(CatchPanicLayer::new())
        .with_state(app_state);

    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{port}"))
        .await
        .unwrap();
    axum::serve(listener, app).await.unwrap();
}

#[derive(Deserialize)]
struct FindPathReq {
    plane: i32,
    start: Point,
    end: Point,
    algo: Option<Algo>,
}

#[derive(Serialize, new)]
struct FindPathRes {
    size: i32,
    path: Vec<Point>,
}

async fn find_path(state: State<AppState>, Json(req): Json<FindPathReq>) -> Response {
    info!(
        "find-path, start: {} end: {} plane: {} algo: {:?}",
        req.start, req.end, req.plane, req.algo
    );

    let grid = state.tile_pathfinder.get_plane(req.plane as usize);

    match grid.find_path(&req.start, &req.end, req.algo.unwrap_or(Algo::AStar)) {
        Ok(Some(path)) => (
            StatusCode::OK,
            Json(FindPathRes::new((path.len() - 1) as i32, minify_path(path))),
        )
            .into_response(),
        Ok(None) => (StatusCode::BAD_REQUEST, "No path").into_response(),
        Err(e) => (StatusCode::BAD_REQUEST, format!("{e}")).into_response(),
    }
}

#[derive(Deserialize)]
struct FindDistancesReq {
    plane: i32,
    start: Point,
    ends: Vec<Point>,
}

#[derive(Serialize, new)]
struct FindDistancesRes {
    distances: Vec<Distance>,
}

#[derive(Serialize, new)]
struct Distance {
    point: Point,
    distance: i32,
}

async fn find_distances(state: State<AppState>, Json(req): Json<FindDistancesReq>) -> Response {
    info!("find-distances, start: {} plane: {}", req.start, req.plane);

    let grid = state.tile_pathfinder.get_plane(req.plane as usize);

    let distances_result = grid.find_distances(&req.start, req.ends);
    match distances_result {
        Ok(distances) => {
            let distances = distances
                .into_iter()
                .map(|(point, distance)| Distance::new(point, distance))
                .collect();
            (StatusCode::OK, Json(FindDistancesRes::new(distances))).into_response()
        }
        Err(e) => (StatusCode::BAD_REQUEST, format!("{e}")).into_response(),
    }
}
