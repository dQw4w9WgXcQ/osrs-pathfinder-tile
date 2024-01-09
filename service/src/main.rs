use std::collections::HashMap;

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::{routing::post, Json, Router};
use derive_new::new;
use serde::{Deserialize, Serialize};

use osrs_pathfinder_tile::{Point, TilePathfinder};

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
    tracing_subscriber::fmt::init();

    let tile_pathfinder = TilePathfinder::load("./grid.zip").unwrap();

    let app_state = AppState::new(tile_pathfinder);

    let app = Router::new()
        .route("/find-path", post(find_path))
        .route("/find-distances", post(find_distances))
        .with_state(app_state);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

#[derive(Serialize, Deserialize, PartialEq, Eq, Hash, new)]
struct PointDto {
    x: i32,
    y: i32,
}

impl PointDto {
    fn to_point(&self) -> Point {
        Point::new(self.x, self.y)
    }
}

#[derive(Deserialize)]
struct FindPathReq {
    plane: i32,
    start: PointDto,
    end: PointDto,
}

#[derive(Serialize, new)]
struct FindPathRes {
    path: Option<Vec<PointDto>>,
}

async fn find_path(state: State<AppState>, Json(req): Json<FindPathReq>) -> Response {
    let grid = state.tile_pathfinder.get_plane(req.plane as usize);

    let start = req.start.to_point();
    let end = req.end.to_point();

    match grid.find_path(&start, &end) {
        Ok(path_opt) => {
            if path_opt.is_none() {
                return (StatusCode::OK, Json(FindPathRes::new(None))).into_response();
            }

            let path = path_opt.unwrap();
            let path_dto = path.iter().map(|p| PointDto::new(p.x, p.y)).collect();

            (StatusCode::OK, Json(FindPathRes::new(Some(path_dto)))).into_response()
        }
        Err(e) => (StatusCode::BAD_REQUEST, e).into_response(),
    }
}

#[derive(Deserialize)]
struct FindDistancesReq {
    plane: i32,
    origin: PointDto,
    destinations: Vec<PointDto>,
}

#[derive(Serialize, new)]
struct FindDistancesRes {
    distances: HashMap<PointDto, i32>,
}

async fn find_distances(state: State<AppState>, Json(req): Json<FindDistancesReq>) -> Response {
    let grid = state.tile_pathfinder.get_plane(req.plane as usize);

    let origin = req.origin.to_point();
    let destinations = req.destinations.iter().map(|p| p.to_point()).collect();

    println!("origin: {:?}", origin);

    match grid.find_distances(&origin, destinations) {
        Ok(distances) => {
            let distances_dto = distances
                .iter()
                .map(|(p, d)| (PointDto::new(p.x, p.y), *d))
                .collect();

            (StatusCode::OK, Json(FindDistancesRes::new(distances_dto))).into_response()
        }
        Err(e) => (StatusCode::BAD_REQUEST, e).into_response(),
    }
}
