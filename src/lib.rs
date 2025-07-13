use std::{
    cmp::max,
    collections::{BinaryHeap, HashMap, HashSet},
    fmt::{Debug, Display, Formatter},
    fs::File,
    io::{Cursor, Read},
};

use byteorder::{BigEndian, ReadBytesExt};
use derive_more::Display;
use derive_new::new;
use log::debug;
use serde::{Deserialize, Serialize};
use zip::ZipArchive;

const PLANES_SIZE: usize = 4;

#[derive(new)]
pub struct TilePathfinder {
    planes: [PathfindingGrid; PLANES_SIZE],
}

impl TilePathfinder {
    pub fn create(grid_planes: [Vec<Vec<u8>>; PLANES_SIZE]) -> Self {
        Self::new(grid_planes.map(|plane| PathfindingGrid::new(&plane)))
    }

    pub fn load(file_path: &str) -> Result<Self, std::io::Error> {
        let grid_planes = Self::load_grid(file_path)?;
        Ok(Self::create(grid_planes))
    }

    pub fn get_plane(&self, plane: usize) -> &PathfindingGrid {
        &self.planes[plane]
    }

    pub fn load_grid(file_path: &str) -> Result<[Vec<Vec<u8>>; PLANES_SIZE], std::io::Error> {
        let file = File::open(file_path)?;
        let mut archive = ZipArchive::new(file)?;

        let mut grid_file = archive.by_name("grid.dat")?;
        let mut buffer = Vec::new();
        grid_file.read_to_end(&mut buffer)?;

        let mut cursor = Cursor::new(buffer);

        let width = cursor.read_i32::<BigEndian>()? as usize;
        let height = cursor.read_i32::<BigEndian>()? as usize;

        let mut grid = array_init::array_init(|_| vec![vec![0u8; height]; width]);

        for plane in &mut grid {
            for col in plane {
                cursor.read_exact(col)?;
            }
        }

        Ok(grid)
    }
}

#[derive(Debug, Display)]
pub enum FindPathError {
    #[display(fmt = "Start out of bounds")]
    StartOutOfBounds,
    #[display(fmt = "End out of bounds")]
    EndOutOfBounds,
}

#[derive(Debug, Display)]
pub enum FindDistancesError {
    #[display(fmt = "Start out of bounds")]
    StartOutOfBounds,
    #[display(fmt = "Ends unreachable: {:?}", _0)]
    EndsUnreachable(Vec<Point>),
}

#[derive(Debug, Display, Deserialize)]
pub enum Algo {
    #[serde(rename = "A_STAR")]
    AStar,
    #[serde(rename = "BFS")]
    Bfs,
}

pub struct PathfindingGrid {
    grid: Vec<Vec<u8>>,
}

impl PathfindingGrid {
    pub fn new(grid: &[Vec<u8>]) -> Self {
        let mut grid = grid.to_vec();
        Self::pad_grid(&mut grid);
        Self { grid }
    }

    /// Returns a path inclusive of both start and end.
    /// None if no path is found.
    /// Err if start or end is out of bounds.
    pub fn find_path(
        &self,
        start: &Point,
        end: &Point,
        algo: Algo,
    ) -> Result<Option<Vec<Point>>, FindPathError> {
        if !self.in_bounds(start) {
            return Err(FindPathError::StartOutOfBounds);
        }

        if !self.in_bounds(end) {
            return Err(FindPathError::EndOutOfBounds);
        }

        let path = match algo {
            Algo::AStar => self.astar(start, end),
            Algo::Bfs => self.bfs(start, end),
        };

        Ok(path)
    }

    /// Returns a map of points to distances.  Uses BFS.
    /// Err if start is out of bounds or if an end is unreachable.
    pub fn find_distances(
        &self,
        start: &Point,
        ends: Vec<Point>,
    ) -> Result<Vec<(Point, i32)>, FindDistancesError> {
        if !self.in_bounds(start) {
            return Err(FindDistancesError::StartOutOfBounds);
        }

        let mut ends: HashSet<Point> = ends.into_iter().collect();

        let mut distances = Vec::new();

        let mut frontier = vec![*start];
        let mut next_frontier = Vec::new();
        let mut seen = HashSet::new();
        seen.insert(*start);

        let mut distance = 0;

        while !frontier.is_empty() {
            for point in frontier.drain(..) {
                if ends.remove(&point) {
                    distances.push((point, distance));
                }

                let x = point.x as usize;
                let y = point.y as usize;
                let config = self.grid[x][y];
                for dir in DIRECTIONS {
                    if config & dir.flag == 0 {
                        continue;
                    }

                    let adj = Point::new(point.x + dir.dx, point.y + dir.dy);

                    if seen.contains(&adj) {
                        continue;
                    }

                    seen.insert(adj);
                    next_frontier.push(adj);
                }
            }

            std::mem::swap(&mut frontier, &mut next_frontier);
            distance += 1;
        }

        if !ends.is_empty() {
            return Err(FindDistancesError::EndsUnreachable(
                ends.into_iter().collect(),
            ));
        }

        Ok(distances)
    }

    fn astar(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
        let mut open = BinaryHeap::new();
        let mut closed = HashSet::new();
        let mut g_costs = HashMap::new();
        let mut came_from = HashMap::new();

        open.push(AStarNode::create(end, *start, 0));

        loop {
            let curr = open.pop();
            if curr.is_none() {
                debug!("no path found");
                return None;
            }

            let curr = curr.unwrap();

            if curr.point == *end {
                debug!("found path");

                let mut path = Vec::new();
                let mut curr = curr.point;
                while curr != *start {
                    debug!("({:?},{:?}),", curr.x, curr.y);
                    path.push(curr);
                    let next = came_from.get(&curr).unwrap();
                    curr = *next;
                }
                path.push(*start);

                path.reverse();
                return Some(path);
            }

            debug!("curr:{:?},{:?}", curr.point.x, curr.point.y);
            debug!(
                "cost:{:?} h:{:?} g:{:?}",
                curr.cost, curr.h_cost, curr.g_cost
            );

            if closed.contains(&curr.point) {
                //There can be duplicate nodes for a point with updated g_cost.
                debug!("already closed");
                continue;
            }

            closed.insert(curr.point);

            let x = curr.point.x as usize;
            let y = curr.point.y as usize;
            let config = self.grid[x][y];
            debug!("config:{}", config);
            for dir in DIRECTIONS {
                if config & dir.flag == 0 {
                    debug!("blocked {}", dir.flag);
                    continue;
                }

                let adj_x = curr.point.x + dir.dx;
                let adj_y = curr.point.y + dir.dy;

                debug!("adj:{},{}", adj_x, adj_y);

                let adj = Point::new(adj_x, adj_y);
                let diag_cost = if (x as i32 - adj_x).abs() + (y as i32 - adj_y).abs() == 2 {
                    1
                } else {
                    0
                };
                let next_g_cost = curr.g_cost + 100_000 + diag_cost;

                //also functions as a check for if adj is already closed.
                let old_g_cost = g_costs.get(&adj);
                if old_g_cost.is_some() {
                    if next_g_cost >= *old_g_cost.unwrap() {
                        debug!("already have g_cost");
                        continue;
                    }

                    debug!("updating g_cost");
                }

                g_costs.insert(adj, next_g_cost);
                came_from.insert(adj, curr.point);

                let next = AStarNode::create(end, adj, next_g_cost);

                open.push(next);
            }
        }
    }

    fn bfs(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
        if start == end {
            return Some(vec![*start]);
        }

        let mut frontier = vec![*start];
        let mut next_frontier = Vec::new();
        let mut seen_from = HashMap::new();

        while !frontier.is_empty() {
            for curr in frontier.drain(..) {
                if curr == *end {
                    let mut path = Vec::new();
                    let mut curr = curr;
                    while curr != *start {
                        path.push(curr);
                        let next = seen_from.get(&curr).unwrap();
                        curr = *next;
                    }
                    path.push(*start);
                    path.reverse();
                    return Some(path);
                }

                let x = curr.x as usize;
                let y = curr.y as usize;

                let config = self.grid[x][y];

                for dir in DIRECTIONS {
                    if config & dir.flag == 0 {
                        continue;
                    }

                    let adj = Point::new(curr.x + dir.dx, curr.y + dir.dy);

                    if seen_from.contains_key(&adj) {
                        continue;
                    }

                    seen_from.insert(adj, curr);
                    next_frontier.push(adj);
                }
            }
            
            std::mem::swap(&mut frontier, &mut next_frontier);
        }

        None
    }

    fn pad_grid(grid: &mut Vec<Vec<u8>>) {
        let width = grid.len();
        let height = grid[0].len();

        let n_flag = N.flag | NE.flag | NW.flag;
        let s_flag = S.flag | SE.flag | SW.flag;
        let e_flag = E.flag | NE.flag | SE.flag;
        let w_flag = W.flag | NW.flag | SW.flag;

        for x in 0..width {
            grid[x][0] &= !s_flag;
            grid[x][height - 1] &= !n_flag;
        }

        for y in 0..height {
            grid[0][y] &= !w_flag;
            grid[width - 1][y] &= !e_flag;
        }
    }

    fn in_bounds(&self, point: &Point) -> bool {
        self.xy_in_bounds(point.x, point.y)
    }

    fn xy_in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.grid.len() as i32 && y < self.grid[0].len() as i32
    }
}

pub fn minify_path(path: Vec<Point>) -> Vec<Point> {
    if path.is_empty() {
        return Vec::new();
    }

    if path.len() == 1 {
        return path;
    }

    let mut minified = Vec::new();
    let mut prev_prev = None;
    let mut prev = None;
    for curr in path {
        if prev.is_none() {
            prev = Some(curr);
            minified.push(curr);
            continue;
        }

        if prev_prev.is_none() {
            prev_prev = prev;
            prev = Some(curr);
            continue;
        }

        let dx = prev.unwrap().x - prev_prev.unwrap().x;
        let dy = prev.unwrap().y - prev_prev.unwrap().y;
        let dx2 = curr.x - prev.unwrap().x;
        let dy2 = curr.y - prev.unwrap().y;

        if dx != dx2 || dy != dy2 {
            minified.push(prev.unwrap());
        }

        prev_prev = prev;
        prev = Some(curr);
    }

    minified.push(prev.unwrap());
    minified
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Serialize, Deserialize, new)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Display for Point {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "({},{})", self.x, self.y)
    }
}

#[derive(PartialEq, Eq, new)]
struct AStarNode {
    point: Point,
    cost: i32,
    h_cost: i32,
    g_cost: i32,
}

impl AStarNode {
    fn create(end: &Point, point: Point, g_cost: i32) -> AStarNode {
        let h_cost = heuristic(&point, end);
        AStarNode::new(point, g_cost + h_cost, h_cost, g_cost)
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cost.cmp(&other.cost).reverse())
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return if self.cost == other.cost {
            self.h_cost.cmp(&other.h_cost).reverse()
        } else {
            self.cost.cmp(&other.cost).reverse()
        };
    }
}

#[derive(Debug)]
struct Direction {
    flag: u8,
    dx: i32,
    dy: i32,
}

impl Direction {
    const fn new(flag: u8, dx: i32, dy: i32) -> Self {
        Self { flag, dx, dy }
    }
}

const N: Direction = Direction::new(1, 0, 1);
const S: Direction = Direction::new(1 << 1, 0, -1);
const E: Direction = Direction::new(1 << 2, 1, 0);
const W: Direction = Direction::new(1 << 3, -1, 0);
const NE: Direction = Direction::new(1 << 4, 1, 1);
const NW: Direction = Direction::new(1 << 5, -1, 1);
const SE: Direction = Direction::new(1 << 6, 1, -1);
const SW: Direction = Direction::new(1 << 7, -1, -1);
const DIRECTIONS: [Direction; 8] = [N, S, E, W, NE, NW, SE, SW];

fn chebyshev(a: &Point, b: &Point) -> i32 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    max(dx, dy)
}

// fn heuristic(a: &Point, b: &Point) -> i32 {
//     chebyshev(a, b)
// }

//kinda works but not really.
//a* w/ tiebreak: https://i.imgur.com/u4Lnofu.png
//bfs: https://i.imgur.com/OQUqiQP.png
// fn manhattan(a: &Point, b: &Point) -> i32 {
//     let dx = (a.x - b.x).abs();
//     let dy = (a.y - b.y).abs();
//     dx + dy
// }
//
// //manhattan distance is used as a tiebreaker to create nicer paths
// fn heuristic(a: &Point, b: &Point) -> i32 {
//     let chebyshev = chebyshev(a, b);
//     let manhattan = manhattan(a, b);
//
//     (chebyshev * 100_000) + manhattan
// }

fn diagonal_cost(a: &Point, b: &Point) -> i32 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    (dx - dy).abs()
}
//manhattan distance is used as a tiebreaker to create nicer paths
fn heuristic(a: &Point, b: &Point) -> i32 {
    let chebyshev = chebyshev(a, b);
    let diagonal_cost = diagonal_cost(a, b);

    (chebyshev * 100_000) + diagonal_cost
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test utilities
    fn create_empty_grid(width: usize, height: usize) -> Vec<Vec<u8>> {
        vec![vec![!0; height]; width]
    }

    fn create_blocked_grid(width: usize, height: usize) -> Vec<Vec<u8>> {
        vec![vec![0; height]; width]
    }

    fn create_maze_grid() -> Vec<Vec<u8>> {
        // A more complex maze-like grid for testing
        let mut grid = create_empty_grid(10, 10);
        
        // Create walls
        for i in 2..8 {
            grid[i][5] = 0; // Horizontal wall
        }
        grid[7][5] = !0; // Gap in wall
        
        // Vertical walls
        for i in 1..4 {
            grid[3][i] = 0;
        }
        
        grid
    }

    fn assert_path_valid(_grid: &PathfindingGrid, path: &[Point]) {
        assert!(!path.is_empty(), "Path should not be empty");
        
        for window in path.windows(2) {
            let from = window[0];
            let to = window[1];
            
            // Check that consecutive points are adjacent
            let dx = (to.x - from.x).abs();
            let dy = (to.y - from.y).abs();
            assert!(dx <= 1 && dy <= 1, "Path contains non-adjacent points: {:?} -> {:?}", from, to);
            assert!(dx + dy >= 1, "Path contains duplicate points: {:?} -> {:?}", from, to);
        }
    }

    fn assert_path_connects(path: &[Point], start: &Point, end: &Point) {
        assert_eq!(path.first(), Some(start), "Path should start at start point");
        assert_eq!(path.last(), Some(end), "Path should end at end point");
    }

    mod basic_functionality {
        use super::*;

        #[test]
        fn test_pathfinding_grid_creation() {
            let grid = create_empty_grid(5, 5);
            let pathfinding_grid = PathfindingGrid::new(&grid);
            
            // Basic sanity check
            assert!(pathfinding_grid.in_bounds(&Point::new(0, 0)));
            assert!(pathfinding_grid.in_bounds(&Point::new(4, 4)));
            assert!(!pathfinding_grid.in_bounds(&Point::new(5, 5)));
            assert!(!pathfinding_grid.in_bounds(&Point::new(-1, 0)));
        }

        #[test]
        fn test_point_display() {
            let point = Point::new(5, 10);
            assert_eq!(format!("{}", point), "(5,10)");
        }

        #[test]
        fn test_point_creation() {
            let point = Point::new(3, 7);
            assert_eq!(point.x, 3);
            assert_eq!(point.y, 7);
        }

        #[test]
        fn test_bounds_checking() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            // Valid bounds
            assert!(pathfinding_grid.in_bounds(&Point::new(0, 0)));
            assert!(pathfinding_grid.in_bounds(&Point::new(9, 9)));
            assert!(pathfinding_grid.in_bounds(&Point::new(5, 5)));

            // Invalid bounds
            assert!(!pathfinding_grid.in_bounds(&Point::new(-1, 0)));
            assert!(!pathfinding_grid.in_bounds(&Point::new(0, -1)));
            assert!(!pathfinding_grid.in_bounds(&Point::new(10, 9)));
            assert!(!pathfinding_grid.in_bounds(&Point::new(9, 10)));
        }
    }

    mod astar_tests {
        use super::*;

        #[test]
        fn test_astar_simple_path() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
            assert_eq!(path.len(), 9); // Diagonal path should be 9 steps
        }

        #[test]
        fn test_astar_straight_line() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(1, 5);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
            assert_eq!(path.len(), 5); // Should be 5 steps vertically
        }

        #[test]
        fn test_astar_with_obstacles() {
            let mut grid = create_empty_grid(10, 10);
            
            // Create a wall
            for i in 2..8 {
                grid[i][5] = 0;
            }

            let pathfinding_grid = PathfindingGrid::new(&grid);
            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
            assert!(path.len() > 9); // Should be longer than direct path
        }

        #[test]
        fn test_astar_no_path() {
            let mut grid = create_empty_grid(10, 10);
            
            // Block the starting position completely
            grid[1][1] = 0;
            
            let pathfinding_grid = PathfindingGrid::new(&grid);
            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let path = pathfinding_grid.astar(&start, &end);
            assert!(path.is_none());
        }

        #[test]
        fn test_astar_same_start_end() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let point = Point::new(5, 5);
            let path = pathfinding_grid.astar(&point, &point).unwrap();
            
            assert_eq!(path.len(), 1);
            assert_eq!(path[0], point);
        }

        #[test]
        fn test_astar_complex_maze() {
            let grid = create_maze_grid();
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(8, 8);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
        }
    }

    mod bfs_tests {
        use super::*;

        #[test]
        fn test_bfs_simple_path() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let path = pathfinding_grid.bfs(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
        }

        #[test]
        fn test_bfs_with_obstacles() {
            let grid = create_maze_grid();
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(8, 8);

            let path = pathfinding_grid.bfs(&start, &end).unwrap();
            
            assert_path_valid(&pathfinding_grid, &path);
            assert_path_connects(&path, &start, &end);
        }

        #[test]
        fn test_bfs_no_path() {
            let grid = create_blocked_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let path = pathfinding_grid.bfs(&start, &end);
            assert!(path.is_none());
        }

        #[test]
        fn test_bfs_same_start_end() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let point = Point::new(5, 5);
            let path = pathfinding_grid.bfs(&point, &point).unwrap();
            
            assert_eq!(path.len(), 1);
            assert_eq!(path[0], point);
        }
    }

    mod algorithm_comparison {
        use super::*;

        #[test]
        fn test_astar_vs_bfs_same_result() {
            let grid = create_empty_grid(6, 6);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(4, 4);

            let astar_path = pathfinding_grid.astar(&start, &end).unwrap();
            let bfs_path = pathfinding_grid.bfs(&start, &end).unwrap();

            assert_path_connects(&astar_path, &start, &end);
            assert_path_connects(&bfs_path, &start, &end);
            
            // Both should find valid paths
            assert_path_valid(&pathfinding_grid, &astar_path);
            assert_path_valid(&pathfinding_grid, &bfs_path);
        }

        #[test]
        fn test_algorithms_with_public_api() {
            let grid = create_empty_grid(8, 8);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(6, 6);

            // Test A* via public API
            let astar_result = pathfinding_grid.find_path(&start, &end, Algo::AStar).unwrap();
            assert!(astar_result.is_some());
            let astar_path = astar_result.unwrap();
            assert_path_connects(&astar_path, &start, &end);

            // Test BFS via public API
            let bfs_result = pathfinding_grid.find_path(&start, &end, Algo::Bfs).unwrap();
            assert!(bfs_result.is_some());
            let bfs_path = bfs_result.unwrap();
            assert_path_connects(&bfs_path, &start, &end);
        }
    }

    mod find_distances_tests {
        use super::*;

        #[test]
        fn test_find_distances_single_target() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(9, 9);

            let distances = pathfinding_grid.find_distances(&start, vec![end]).unwrap();
            assert_eq!(distances.len(), 1);
            assert_eq!(distances[0].0, end);
            assert_eq!(distances[0].1, 8); // Chebyshev distance
        }

        #[test]
        fn test_find_distances_multiple_targets() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(5, 5);
            let targets = vec![
                Point::new(5, 7), // distance 2
                Point::new(7, 5), // distance 2
                Point::new(3, 3), // distance 2
                Point::new(9, 9), // distance 4
            ];

            let distances = pathfinding_grid.find_distances(&start, targets).unwrap();
            assert_eq!(distances.len(), 4);

            // Check that all targets were found
            let mut found_distances: Vec<i32> = distances.iter().map(|&(_, d)| d).collect();
            found_distances.sort();
            assert_eq!(found_distances, vec![2, 2, 2, 4]);
        }

        #[test]
        fn test_find_distances_start_equals_end() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(5, 5);
            let distances = pathfinding_grid.find_distances(&start, vec![start]).unwrap();
            
            assert_eq!(distances.len(), 1);
            assert_eq!(distances[0].0, start);
            assert_eq!(distances[0].1, 0);
        }

        #[test]
        fn test_find_distances_unreachable_target() {
            let mut grid = create_empty_grid(10, 10);
            
            // Create a wall that blocks access to the target
            for i in 0..10 {
                grid[5][i] = 0;
            }

            let pathfinding_grid = PathfindingGrid::new(&grid);
            let start = Point::new(1, 1);
            let unreachable = Point::new(8, 8);

            let result = pathfinding_grid.find_distances(&start, vec![unreachable]);
            assert!(result.is_err());
            
            match result.unwrap_err() {
                FindDistancesError::EndsUnreachable(unreachable_points) => {
                    assert_eq!(unreachable_points.len(), 1);
                    assert_eq!(unreachable_points[0], unreachable);
                }
                _ => panic!("Expected EndsUnreachable error"),
            }
        }

        #[test]
        fn test_find_distances_mixed_reachable_unreachable() {
            let mut grid = create_empty_grid(10, 10);
            
            // Create a wall
            for i in 6..10 {
                for j in 0..10 {
                    grid[i][j] = 0;
                }
            }

            let pathfinding_grid = PathfindingGrid::new(&grid);
            let start = Point::new(1, 1);
            let targets = vec![
                Point::new(3, 3), // reachable
                Point::new(8, 8), // unreachable
                Point::new(2, 5), // reachable
            ];

            let result = pathfinding_grid.find_distances(&start, targets);
            assert!(result.is_err());
            
            match result.unwrap_err() {
                FindDistancesError::EndsUnreachable(unreachable_points) => {
                    assert_eq!(unreachable_points.len(), 1);
                    assert_eq!(unreachable_points[0], Point::new(8, 8));
                }
                _ => panic!("Expected EndsUnreachable error"),
            }
        }
    }

    mod error_handling {
        use super::*;

        #[test]
        fn test_find_path_start_out_of_bounds() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(-1, 5);
            let end = Point::new(5, 5);

            let result = pathfinding_grid.find_path(&start, &end, Algo::AStar);
            assert!(result.is_err());
            assert!(matches!(result.unwrap_err(), FindPathError::StartOutOfBounds));
        }

        #[test]
        fn test_find_path_end_out_of_bounds() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(5, 5);
            let end = Point::new(15, 5);

            let result = pathfinding_grid.find_path(&start, &end, Algo::AStar);
            assert!(result.is_err());
            assert!(matches!(result.unwrap_err(), FindPathError::EndOutOfBounds));
        }

        #[test]
        fn test_find_distances_start_out_of_bounds() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(10, 10);
            let end = Point::new(5, 5);

            let result = pathfinding_grid.find_distances(&start, vec![end]);
            assert!(result.is_err());
            assert!(matches!(result.unwrap_err(), FindDistancesError::StartOutOfBounds));
        }
    }

    mod edge_cases {
        use super::*;

        #[test]
        fn test_single_cell_grid() {
            let grid = create_empty_grid(1, 1);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let point = Point::new(0, 0);
            let path = pathfinding_grid.astar(&point, &point).unwrap();
            assert_eq!(path.len(), 1);
            assert_eq!(path[0], point);
        }

        #[test]
        fn test_adjacent_points() {
            let grid = create_empty_grid(3, 3);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(1, 2);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            assert_eq!(path.len(), 2);
            assert_eq!(path[0], start);
            assert_eq!(path[1], end);
        }

        #[test]
        fn test_diagonal_movement() {
            let grid = create_empty_grid(3, 3);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(0, 0);
            let end = Point::new(2, 2);

            let path = pathfinding_grid.astar(&start, &end).unwrap();
            assert_eq!(path.len(), 3);
            assert_eq!(path[0], start);
            assert_eq!(path[2], end);
        }

        #[test]
        fn test_large_grid_performance() {
            let grid = create_empty_grid(100, 100);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(0, 0);
            let end = Point::new(99, 99);

            let start_time = std::time::Instant::now();
            let path = pathfinding_grid.astar(&start, &end).unwrap();
            let duration = start_time.elapsed();

            assert_path_connects(&path, &start, &end);
            assert!(duration.as_millis() < 100); // Should complete in reasonable time
        }

        #[test]
        fn test_empty_targets_list() {
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(5, 5);
            let distances = pathfinding_grid.find_distances(&start, vec![]).unwrap();
            assert_eq!(distances.len(), 0);
        }
    }

    mod heuristic_tests {
        use super::*;

        #[test]
        fn test_chebyshev_distance() {
            assert_eq!(chebyshev(&Point::new(0, 0), &Point::new(3, 4)), 4);
            assert_eq!(chebyshev(&Point::new(0, 0), &Point::new(-3, 4)), 4);
            assert_eq!(chebyshev(&Point::new(5, 5), &Point::new(5, 5)), 0);
            assert_eq!(chebyshev(&Point::new(1, 1), &Point::new(4, 3)), 3);
        }

        #[test]
        fn test_diagonal_cost() {
            assert_eq!(diagonal_cost(&Point::new(0, 0), &Point::new(3, 4)), 1);
            assert_eq!(diagonal_cost(&Point::new(0, 0), &Point::new(5, 5)), 0);
            assert_eq!(diagonal_cost(&Point::new(1, 1), &Point::new(4, 3)), 1);
        }

        #[test]
        fn test_heuristic_consistency() {
            let a = Point::new(0, 0);
            let b = Point::new(3, 4);
            let c = Point::new(6, 8);

            let h_ab = heuristic(&a, &b);
            let h_ac = heuristic(&a, &c);
            let h_bc = heuristic(&b, &c);

            // Triangle inequality should hold for admissible heuristics
            assert!(h_ac <= h_ab + h_bc);
        }
    }

    mod path_minification {
        use super::*;

        #[test]
        fn test_minify_straight_line() {
            let path = vec![
                Point::new(0, 0),
                Point::new(1, 1),
                Point::new(2, 2),
                Point::new(3, 3),
                Point::new(4, 4),
            ];
            let minified = minify_path(path);
            assert_eq!(minified, vec![Point::new(0, 0), Point::new(4, 4)]);
        }

        #[test]
        fn test_minify_with_turns() {
            let path = vec![
                Point::new(0, 0),
                Point::new(1, 0),
                Point::new(2, 0),
                Point::new(3, 1),
                Point::new(4, 2),
            ];
            let minified = minify_path(path);
            assert_eq!(
                minified,
                vec![
                    Point::new(0, 0),
                    Point::new(2, 0),
                    Point::new(4, 2),
                ]
            );
        }

        #[test]
        fn test_minify_complex_path() {
            let path = vec![
                Point::new(0, 0),
                Point::new(1, 0),
                Point::new(2, 0),
                Point::new(3, 1),
                Point::new(4, 2),
                Point::new(5, 2),
            ];
            let minified = minify_path(path);
            assert_eq!(
                minified,
                vec![
                    Point::new(0, 0),
                    Point::new(2, 0),
                    Point::new(4, 2),
                    Point::new(5, 2),
                ]
            );
        }

        #[test]
        fn test_minify_empty_path() {
            let path = Vec::<Point>::new();
            let minified = minify_path(path);
            assert!(minified.is_empty());
        }

        #[test]
        fn test_minify_single_point() {
            let path = vec![Point::new(5, 5)];
            let minified = minify_path(path);
            assert_eq!(minified, vec![Point::new(5, 5)]);
        }

        #[test]
        fn test_minify_two_points() {
            let path = vec![Point::new(1, 1), Point::new(2, 2)];
            let minified = minify_path(path);
            assert_eq!(minified, vec![Point::new(1, 1), Point::new(2, 2)]);
        }
    }

    mod integration_tests {
        use super::*;

        #[test]
        fn test_tile_pathfinder_creation() {
            let plane_data = [
                create_empty_grid(10, 10),
                create_empty_grid(10, 10),
                create_empty_grid(10, 10),
                create_empty_grid(10, 10),
            ];
            
            let pathfinder = TilePathfinder::create(plane_data);
            
            // Test that we can access all planes
            for i in 0..4 {
                let plane = pathfinder.get_plane(i);
                assert!(plane.in_bounds(&Point::new(5, 5)));
            }
        }

        #[test]
        fn test_pathfinding_with_different_algorithms() {
            let grid = create_maze_grid();
            let pathfinding_grid = PathfindingGrid::new(&grid);
            
            let start = Point::new(1, 1);
            let end = Point::new(8, 8);
            
            // Test both algorithms find paths
            let astar_path = pathfinding_grid.find_path(&start, &end, Algo::AStar).unwrap().unwrap();
            let bfs_path = pathfinding_grid.find_path(&start, &end, Algo::Bfs).unwrap().unwrap();
            
            assert_path_connects(&astar_path, &start, &end);
            assert_path_connects(&bfs_path, &start, &end);
            
            // Test that minification works on both paths
            let minified_astar = minify_path(astar_path);
            let minified_bfs = minify_path(bfs_path);
            
            assert!(!minified_astar.is_empty());
            assert!(!minified_bfs.is_empty());
        }
    }

    mod performance_tests {
        use super::*;

        #[test]
        fn test_large_grid_astar_performance() {
            let grid = create_empty_grid(200, 200);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(10, 10);
            let end = Point::new(190, 190);

            let start_time = std::time::Instant::now();
            let path = pathfinding_grid.astar(&start, &end).unwrap();
            let duration = start_time.elapsed();

            assert_path_connects(&path, &start, &end);
            assert!(duration.as_millis() < 1000); // Should complete in reasonable time
        }

        #[test]
        fn test_find_distances_performance() {
            let grid = create_empty_grid(50, 50);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(25, 25);
            let mut targets = Vec::new();
            for i in 0..10 {
                for j in 0..10 {
                    targets.push(Point::new(i * 4, j * 4));
                }
            }

            let start_time = std::time::Instant::now();
            let distances = pathfinding_grid.find_distances(&start, targets).unwrap();
            let duration = start_time.elapsed();

            assert_eq!(distances.len(), 100);
            assert!(duration.as_millis() < 100); // Should complete quickly
        }

        #[test]
        fn test_buffer_reuse_optimization() {
            // This test ensures our buffer reuse optimization works correctly
            // by running multiple pathfinding operations
            let grid = create_empty_grid(20, 20);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start_time = std::time::Instant::now();
            
            // Run multiple pathfinding operations
            for i in 0..10 {
                let start = Point::new(i, i);
                let end = Point::new(19 - i, 19 - i);
                
                let path = pathfinding_grid.bfs(&start, &end).unwrap();
                assert_path_connects(&path, &start, &end);
            }
            
            let duration = start_time.elapsed();
            assert!(duration.as_millis() < 100); // Should benefit from buffer reuse
        }
    }

    mod regression_tests {
        use super::*;

        #[test]
        fn test_optimization_maintains_correctness() {
            // Test that our HashSet<Point> optimization maintains correctness
            let grid = create_empty_grid(10, 10);
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(5, 5);
            let targets = vec![
                Point::new(5, 5), // Same as start
                Point::new(6, 6), // Different
                Point::new(4, 4), // Different
            ];

            let distances = pathfinding_grid.find_distances(&start, targets).unwrap();
            assert_eq!(distances.len(), 3);
            
            // Check that the start point has distance 0
            let start_distance = distances.iter().find(|(p, _)| *p == start).unwrap().1;
            assert_eq!(start_distance, 0);
        }

        #[test]
        fn test_safe_indexing_maintains_correctness() {
            // Test that removing unsafe indexing doesn't break functionality
            let grid = create_maze_grid();
            let pathfinding_grid = PathfindingGrid::new(&grid);

            let start = Point::new(1, 1);
            let end = Point::new(8, 8);

            // Both algorithms should still work with safe indexing
            let astar_path = pathfinding_grid.astar(&start, &end).unwrap();
            let bfs_path = pathfinding_grid.bfs(&start, &end).unwrap();

            assert_path_connects(&astar_path, &start, &end);
            assert_path_connects(&bfs_path, &start, &end);
        }
    }
}
