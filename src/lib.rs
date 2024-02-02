use std::{
    cmp::max,
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
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
        Self::new(grid_planes.map(|plane| PathfindingGrid::new(plane)))
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

pub struct PathfindingGrid {
    grid: Vec<Vec<u8>>,
}

impl PathfindingGrid {
    pub fn new(grid: Vec<Vec<u8>>) -> Self {
        let mut grid = grid;
        Self::pad_grid(&mut grid);
        Self { grid }
    }

    /**
     * Returns a path exclusive of start, inclusive of end.  Uses A*.
     * None if no path is found.
     * Err if start or end is out of bounds.
     */
    pub fn find_path(
        &self,
        start: &Point,
        end: &Point,
    ) -> Result<Option<Vec<Point>>, FindPathError> {
        if !self.in_bounds(start) {
            return Err(FindPathError::StartOutOfBounds);
        }

        if !self.in_bounds(end) {
            return Err(FindPathError::EndOutOfBounds);
        }

        let path = self.bfs(start, end);

        Ok(path)
    }

    /**
     * Returns a map of points to distances.  Uses BFS.
     * Err if start is out of bounds or if an end is unreachable.
     */
    pub fn find_distances(
        &self,
        start: &Point,
        ends: Vec<Point>,
    ) -> Result<Vec<(Point, i32)>, FindDistancesError> {
        if !self.in_bounds(start) {
            return Err(FindDistancesError::StartOutOfBounds);
        }

        let mut ends = ends.iter().collect::<HashSet<&Point>>();

        let mut distances = Vec::new();

        let mut frontier = Vec::new();
        frontier.push(*start);
        let mut seen = HashSet::new();
        seen.insert(*start);

        let mut distance = 0;

        while !frontier.is_empty() {
            let mut next_frontier = Vec::new();
            for point in frontier {
                if ends.contains(&point) {
                    ends.remove(&point);
                    distances.push((point, distance));
                }

                let x = point.x as usize;
                let y = point.y as usize;
                let config = *unsafe { self.grid.get_unchecked(x).get_unchecked(y) };
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

            frontier = next_frontier;

            distance += 1;
        }

        if !ends.is_empty() {
            return Err(FindDistancesError::EndsUnreachable(
                ends.iter().map(|p| **p).collect(),
            ));
        }

        Ok(distances)
    }

    #[allow(dead_code)]
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
            let config = *unsafe { self.grid.get_unchecked(x).get_unchecked(y) };
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
                let next_g_cost = curr.g_cost + 1;

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

    //was used for performance comparison
    #[allow(dead_code)]
    fn bfs(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
        if start == end {
            return Some(vec![*start]);
        }

        let mut frontier = VecDeque::new();
        let mut seen_from = HashMap::new();

        frontier.push_back(*start);
        while !frontier.is_empty() {
            let curr = frontier.pop_front().unwrap();
            if curr == *end {
                let mut path = Vec::new();
                let mut curr = curr;
                while curr != *start {
                    path.push(curr);
                    let next = seen_from.get(&curr).unwrap();
                    curr = *next;
                }
                path.reverse();
                return Some(path);
            }

            let x = curr.x as usize;
            let y = curr.y as usize;

            let config = *unsafe { self.grid.get_unchecked(x).get_unchecked(y) };

            for dir in DIRECTIONS {
                if config & dir.flag == 0 {
                    continue;
                }

                let adj = Point::new(curr.x + dir.dx, curr.y + dir.dy);

                if seen_from.contains_key(&adj) {
                    continue;
                }

                seen_from.insert(adj, curr);
                frontier.push_back(adj);
            }
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

fn heuristic(a: &Point, b: &Point) -> i32 {
    chebyshev(a, b)
}

//doesnt work

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
//     (chebyshev + 100_000) + manhattan
// }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_path() {
        let grid = vec![vec![!0; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);

        let path = pathfinding_grid.astar(&start, &end);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 8);
    }

    #[test]
    fn test_wall() {
        let mut grid = vec![vec![255; 10]; 10];
        grid[1][1] &= !NE.flag;

        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);

        let path = pathfinding_grid.astar(&start, &end);
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 9);
    }

    #[test]
    fn test_wall2() {
        let mut grid = vec![vec![!0; 10]; 10];
        grid[1][1] &= !NE.flag;

        for i in 2..8 {
            grid[i][5] &= !NE.flag & !N.flag & !NW.flag;
        }

        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);

        let path = pathfinding_grid.astar(&start, &end);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 11);
    }

    #[test]
    fn test_no_path() {
        let mut grid = vec![vec![!0; 10]; 10];
        grid[1][1] = 0;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 3);

        let path = pathfinding_grid.astar(&start, &end);

        assert!(path.is_none());
    }

    #[test]
    fn test_chebyshev() {
        let a = chebyshev(&Point::new(0, 0), &Point::new(3, 4));
        assert_eq!(a, 4);
        let b = chebyshev(&Point::new(0, 0), &Point::new(-3, 4));
        assert_eq!(a, b);
    }

    #[test]
    fn test_find_distances() {
        let grid = vec![vec![!0; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let mut ends = Vec::new();
        ends.push(Point::new(9, 9));

        let distances = pathfinding_grid.find_distances(&start, ends).unwrap();

        assert_eq!(distances.len(), 1);

        let distance = distances.get(0).unwrap().1;
        assert_eq!(distance, 8);
    }

    #[test]
    fn test_minify_path() {
        assert_eq!(
            vec![Point { x: 0, y: 0 }, Point { x: 4, y: 4 }],
            minify_path(vec![
                Point { x: 0, y: 0 },
                Point { x: 1, y: 1 },
                Point { x: 2, y: 2 },
                Point { x: 3, y: 3 },
                Point { x: 4, y: 4 },
            ])
        );

        assert_eq!(
            vec![
                Point { x: 0, y: 0 },
                Point { x: 2, y: 0 },
                Point { x: 4, y: 2 },
            ],
            minify_path(vec![
                Point { x: 0, y: 0 },
                Point { x: 1, y: 0 },
                Point { x: 2, y: 0 },
                Point { x: 3, y: 1 },
                Point { x: 4, y: 2 },
            ])
        );

        assert_eq!(
            vec![
                Point { x: 0, y: 0 },
                Point { x: 2, y: 0 },
                Point { x: 4, y: 2 },
                Point { x: 5, y: 2 },
            ],
            minify_path(vec![
                Point { x: 0, y: 0 },
                Point { x: 1, y: 0 },
                Point { x: 2, y: 0 },
                Point { x: 3, y: 1 },
                Point { x: 4, y: 2 },
                Point { x: 5, y: 2 },
            ])
        );

        assert_eq!(Vec::<Point>::new(), minify_path(Vec::<Point>::new()));
    }
}
