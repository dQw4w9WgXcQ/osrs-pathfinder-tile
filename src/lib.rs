use std::cmp::max;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::fmt::{Debug, Formatter};

use derive_new::new;
use log::debug;

pub use load::load_grid;
use serde::{Deserialize, Serialize};

mod load;

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
        let grid_planes = load_grid(file_path)?;
        Ok(Self::create(grid_planes))
    }

    pub fn get_plane(&self, plane: usize) -> &PathfindingGrid {
        &self.planes[plane]
    }
}

#[derive(new)]
pub struct PathfindingGrid {
    grid: Vec<Vec<u8>>,
}

//TODO: better error types
impl PathfindingGrid {
    /**
     * Returns a path exclusive of origin, inclusive of destination.  Uses A*.
     * None if no path is found.
     * Err if origin or destination is out of bounds.
     */
    pub fn find_path(&self, start: &Point, end: &Point) -> Result<Option<Vec<Point>>, String> {
        if !self.in_bounds(start) {
            return Err(format!("start out of bounds: {:?}", start));
        }

        if !self.in_bounds(end) {
            return Err(format!("end out of bounds: {:?}", end));
        }

        let path = self.a_star(start, end);

        Ok(path)
    }

    /**
     * Returns a map of points to distances.  Uses BFS.
     * Err if origin/destination is out of bounds or if a destination is unreachable.
     */
    pub fn find_distances(
        &self,
        start: &Point,
        ends: Vec<Point>,
    ) -> Result<HashMap<Point, i32>, String> {
        if !self.in_bounds(start) {
            return Err(format!("start out of bounds: {:?}", start));
        }

        let mut ends = ends.iter().collect::<HashSet<&Point>>();

        let mut queue = VecDeque::new();
        let mut seen = HashSet::new();

        let mut distances = HashMap::new();

        queue.push_back(*start);
        seen.insert(*start);

        let mut distance = 0;

        loop {
            if queue.is_empty() {
                break;
            }

            let curr = queue.clone();
            queue.clear();
            for point in curr {
                if ends.contains(&point) {
                    ends.remove(&point);
                    distances.insert(point, distance);
                }

                let config = self.grid[point.x as usize][point.y as usize];
                for dir in DIRECTIONS {
                    if config & dir.flag == 0 {
                        continue;
                    }

                    let adj = Point::new(point.x + dir.dx, point.y + dir.dy);

                    if !self.in_bounds(&adj) {
                        continue;
                    }

                    if seen.contains(&adj) {
                        continue;
                    }

                    seen.insert(adj);
                    queue.push_back(adj);
                }
            }

            distance += 1;
        }

        if !ends.is_empty() {
            return Err(format!("unreachable destinations: {:?}", ends));
        }

        Ok(distances)
    }

    fn a_star(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
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

            //might remove later.  not needed if the grid's boundary is padded and origin/destination are valid.
            if !self.xy_in_bounds(curr.point.x, curr.point.y) {
                debug!("out of bounds");
                continue;
            }

            //the compiler can infer x,y are in bounds, so no extra bounds checking happens.
            //if in_bounds check is removed, convert to get_unchecked or the compiler will add bounds checking and no performance gain will be seen.
            let config = self.grid[curr.point.x as usize][curr.point.y as usize];
            debug!("config:{}", config);
            for dir in DIRECTIONS {
                if config & dir.flag == 0 {
                    debug!("blocked {}", dir.flag);
                    continue;
                }

                let adj_x = curr.point.x + dir.dx;
                let adj_y = curr.point.y + dir.dy;

                debug!("adj:{},{}", adj_x, adj_y);

                //the adj can be out of bounds, but will be checked before use.
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

    fn in_bounds(&self, point: &Point) -> bool {
        self.xy_in_bounds(point.x, point.y)
    }

    fn xy_in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.grid.len() as i32 && y < self.grid[0].len() as i32
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, new)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Debug for Point {
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
    fn create(destination: &Point, point: Point, g_cost: i32) -> AStarNode {
        let h_cost = chebychev(&point, destination);
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
const DIRECTIONS: [Direction; 8] = [NE, NW, SE, SW, N, S, E, W];

//heuristic
fn chebychev(a: &Point, b: &Point) -> i32 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    max(dx, dy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_path() {
        let grid = vec![vec![0; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 9);

        let path = pathfinding_grid.a_star(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 8);
    }

    #[test]
    fn test_wall() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = NE.flag;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 9);

        let path = pathfinding_grid.a_star(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 9);
    }

    #[test]
    fn test_wall2() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = NE.flag;

        for i in 2..8 {
            grid[i][5] = NE.flag | N.flag | NW.flag;
        }

        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 9);

        let path = pathfinding_grid.a_star(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 11);
    }

    #[test]
    fn test_no_path() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = N.flag | S.flag | E.flag | W.flag | NE.flag | NW.flag | SE.flag | SW.flag;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 3);

        let path = pathfinding_grid.a_star(&origin, &destination);

        assert!(path.is_none());
    }

    #[test]
    fn test_chebychev() {
        let a = chebychev(&Point::new(0, 0), &Point::new(3, 4));
        assert_eq!(a, 4);
        let b = chebychev(&Point::new(0, 0), &Point::new(-3, 4));
        assert_eq!(a, b);
    }

    #[test]
    fn test_find_distances() {
        let grid = vec![vec![255; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);
        let mut ends = Vec::new();
        ends.push(end);

        let distances = pathfinding_grid.find_distances(&start, ends).unwrap();

        assert_eq!(distances.len(), 1);

        let distance = distances.get(&end).unwrap();
        assert_eq!(distance, &8);
    }
}
