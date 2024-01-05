use std::cmp::max;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::fs::File;
use std::io;
use std::io::{Cursor, Read};

use array_init::array_init;
use byteorder::{BigEndian, ReadBytesExt};
use log::debug;
use zip::ZipArchive;

const PLANES_SIZE: usize = 4;

pub struct TilePathfinding {
    planes: [PathfindingGrid; PLANES_SIZE],
}

impl TilePathfinding {
    pub fn new(grid_planes: [Vec<Vec<u8>>; PLANES_SIZE]) -> TilePathfinding {
        TilePathfinding {
            planes: grid_planes.map(|plane| PathfindingGrid::new(plane)),
        }
    }

    pub fn get_plane(&self, plane: usize) -> &PathfindingGrid {
        &self.planes[plane]
    }
}

pub struct PathfindingGrid {
    grid: Vec<Vec<u8>>,
}

//TODO: better error types
impl PathfindingGrid {
    pub fn new(grid: Vec<Vec<u8>>) -> PathfindingGrid {
        PathfindingGrid { grid }
    }

    /**
     * Uses A*.  Returns a path exclusive of origin, inclusive of destination.
     * Err if origin or destination is out of bounds.
     */
    pub fn find_path(
        &self,
        origin: &Point,
        destination: &Point,
    ) -> Result<Option<Vec<Point>>, String> {
        if !self.in_bounds(origin.x, origin.y) {
            return Err(format!(
                "origin out of bounds: ({:?},{:?})",
                origin.x, origin.y
            ));
        }

        if !self.in_bounds(destination.x, destination.y) {
            return Err(format!(
                "destination out of bounds: ({:?},{:?}",
                destination.x, destination.y
            ));
        }

        let path = self.a_star(origin, destination);

        Ok(path)
    }

    /**
     * Uses BFS.  Returns a map of points to distances.
     * Err if origin/destination is out of bounds or if a destination is unreachable.
     */
    pub fn find_distances(
        &self,
        origin: &Point,
        destinations: HashSet<Point>,
    ) -> Result<HashMap<Point, i32>, String> {
        if !self.in_bounds(origin.x, origin.y) {
            return Err(format!(
                "origin out of bounds: ({:?},{:?})",
                origin.x, origin.y
            ));
        }

        let mut distances = HashMap::new();

        let mut frontier = VecDeque::new();
        let mut seen = HashSet::new();

        frontier.push_back(*origin);

        let mut distance = 0;

        loop {
            let curr = frontier.pop_front();
            if curr.is_none() {
                for destination in destinations {
                    if !distances.contains_key(&destination) {
                        return Err(format!(
                            "no path to destination: ({:?},{:?})",
                            destination.x, destination.y
                        ));
                    }
                }

                return Ok(distances);
            }

            let curr = curr.unwrap();

            seen.insert(curr);

            if destinations.contains(&curr) {
                distances.insert(curr, distance);
            }

            if !self.in_bounds(curr.x, curr.y) {
                continue;
            }

            let config = self.grid[curr.x as usize][curr.y as usize];
            for dir in BLOCKED_DIRS {
                if config & dir.bitmask != 0 {
                    continue;
                }

                let adj_x = curr.x + dir.dx;
                let adj_y = curr.y + dir.dy;

                let adj = Point::new(adj_x, adj_y);

                if seen.contains(&adj) {
                    continue;
                }

                frontier.push_back(adj);
            }

            distance += 1;
        }
    }

    fn a_star(&self, origin: &Point, destination: &Point) -> Option<Vec<Point>> {
        let mut open = BinaryHeap::new();
        let mut closed = HashSet::new();
        let mut g_costs = HashMap::new();
        let mut came_from = HashMap::new();

        open.push(AStarNode::create(destination, *origin, 0));

        loop {
            let curr = open.pop();
            if curr.is_none() {
                debug!("no path found");
                return None;
            }

            let curr = curr.unwrap();

            if curr.point == *destination {
                debug!("found path");

                let mut path = Vec::new();
                let mut curr = curr.point;
                while curr != *origin {
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
            if !self.in_bounds(curr.point.x, curr.point.y) {
                debug!("out of bounds");
                continue;
            }

            //the compiler can infer x,y are in bounds, so no extra bounds checking happens.
            //if in_bounds check is removed, convert to get_unchecked or the compiler will add bounds checking and no performance gain will be seen.
            let config = self.grid[curr.point.x as usize][curr.point.y as usize];
            debug!("config:{:?}", config);

            for dir in BLOCKED_DIRS {
                if config & dir.bitmask != 0 {
                    debug!("blocked {:?}", dir.bitmask);
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

                let next = AStarNode::create(destination, adj, next_g_cost);

                open.push(next);
            }
        }
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.grid.len() as i32 && y < self.grid[0].len() as i32
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Point {
    x: i32,
    y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

#[derive(PartialEq, Eq)]
struct AStarNode {
    point: Point,
    cost: i32,
    h_cost: i32,
    g_cost: i32,
}

impl AStarNode {
    fn new(point: Point, cost: i32, h_cost: i32, g_cost: i32) -> AStarNode {
        AStarNode {
            point,
            cost,
            h_cost,
            g_cost,
        }
    }

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

struct BlockedDir {
    bitmask: u8,
    dx: i32,
    dy: i32,
}

impl BlockedDir {
    const fn new(config: u8, dx: i32, dy: i32) -> BlockedDir {
        BlockedDir {
            bitmask: config,
            dx,
            dy,
        }
    }
}

const N: BlockedDir = BlockedDir::new(1, 0, 1);
const S: BlockedDir = BlockedDir::new(1 << 1, 0, -1);
const E: BlockedDir = BlockedDir::new(1 << 2, 1, 0);
const W: BlockedDir = BlockedDir::new(1 << 3, -1, 0);
const NE: BlockedDir = BlockedDir::new(1 << 4, 1, 1);
const NW: BlockedDir = BlockedDir::new(1 << 5, -1, 1);
const SE: BlockedDir = BlockedDir::new(1 << 6, 1, -1);
const SW: BlockedDir = BlockedDir::new(1 << 7, -1, -1);
const BLOCKED_DIRS: [BlockedDir; 8] = [NE, NW, SE, SW, N, S, E, W];

//heuristic
fn chebychev(from: &Point, to: &Point) -> i32 {
    let dx = (from.x - to.x).abs();
    let dy = (from.y - to.y).abs();
    max(dx, dy)
}

pub fn load_grid(file_path: &str) -> io::Result<[Vec<Vec<u8>>; PLANES_SIZE]> {
    let file = File::open(file_path)?;
    let mut archive = ZipArchive::new(file)?;
    let mut grid_file = archive.by_name("grid.dat")?;
    let mut buffer = Vec::new();
    grid_file.read_to_end(&mut buffer)?;
    let mut cursor = Cursor::new(buffer);

    let width = cursor.read_i32::<BigEndian>()? as usize;
    let height = cursor.read_i32::<BigEndian>()? as usize;

    println!("width: {} height: {}", width, height);

    let mut grid = array_init(|_| vec![vec![0u8; width]; height]);

    for plane in &mut grid {
        for col in plane {
            cursor.read_exact(col)?;
        }
    }

    Ok(grid)
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
        grid[1][1] = NE.bitmask;
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
        grid[1][1] = NE.bitmask;

        for i in 2..8 {
            grid[i][5] = NE.bitmask | N.bitmask | NW.bitmask;
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
        grid[1][1] = N.bitmask
            | S.bitmask
            | E.bitmask
            | W.bitmask
            | NE.bitmask
            | NW.bitmask
            | SE.bitmask
            | SW.bitmask;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 3);

        let path = pathfinding_grid.a_star(&origin, &destination);

        assert!(path.is_none());
    }

    #[test]
    fn test_chebychev() {
        let first = chebychev(&Point::new(0, 0), &Point::new(3, 4));
        assert_eq!(first, 4);
        let second = chebychev(&Point::new(0, 0), &Point::new(-3, 4));
        assert_eq!(first, second);
    }
}
