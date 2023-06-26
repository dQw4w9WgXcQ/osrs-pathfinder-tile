use std::cmp::max;
use std::collections::{BinaryHeap, HashMap, HashSet};

pub struct PathfindingGrid {
    grid: Vec<Vec<i8>>,
}

impl PathfindingGrid {
    pub fn new(grid: Vec<Vec<i8>>) -> PathfindingGrid {
        PathfindingGrid { grid }
    }

    /**
     * uses A*.  exclusive of origin, inclusive of destination.
     * can panic if the grid boundary is not padded.
     */
    pub fn find_path(&self, origin: &Point, destination: &Point) -> Option<Vec<Point>> {
        let mut open = BinaryHeap::new();
        let mut closed = HashSet::new();
        let mut g_costs = HashMap::new();
        let mut came_from = HashMap::new();

        open.push(Node::create(destination, *origin, 0));

        loop {
            let curr = open.pop();
            if curr.is_none() {
                println!("no path found");
                return None;
            }

            let curr = curr.unwrap();

            if curr.point == *destination {
                println!("found path");

                let mut path = Vec::new();
                let mut curr = curr.point;
                while curr != *origin {
                    println!("({:?},{:?}),", curr.x, curr.y);
                    path.push(curr);
                    let next = came_from.get(&curr).unwrap();
                    curr = *next;
                }

                path.reverse();
                return Some(path);
            }

            println!("curr:{:?},{:?}", curr.point.x, curr.point.y);
            println!(
                "cost:{:?} h:{:?} g:{:?}",
                curr.cost, curr.h_cost, curr.g_cost
            );

            if closed.contains(&curr.point) {
                //There can be duplicate nodes for a point with updated g_cost.
                println!("already closed");
                continue;
            }

            for dir in BLOCKED_FLAGS {
                if self.grid[curr.point.x as usize][curr.point.y as usize] & dir.config != 0 {
                    println!("blocked {:?}", dir.config);
                    continue;
                }

                let adj_x = curr.point.x + dir.dx;
                let adj_y = curr.point.y + dir.dy;

                //todo remove later.  our grid's boundary is expected to be padded.
                if adj_x < 0
                    || adj_y < 0
                    || adj_x >= self.grid.len() as i32
                    || adj_y >= self.grid[0].len() as i32
                {
                    continue;
                }

                println!("adj:{},{}", adj_x, adj_y);

                let adj = Point::new(adj_x, adj_y);
                let next_g_cost = curr.g_cost + 1;

                let old_g_cost = g_costs.get(&adj);
                if old_g_cost.is_some() {
                    if next_g_cost >= *old_g_cost.unwrap() {
                        println!("have better g_cost");
                        continue;
                    }

                    println!("updating g_cost");
                }

                g_costs.insert(adj, next_g_cost);
                came_from.insert(adj, curr.point);

                let next = Node::create(destination, adj, next_g_cost);

                open.push(next);
            }

            closed.insert(curr.point);
            println!("closed:{:?},{:?}", curr.point.x, curr.point.y);
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Point {
    x: i32,
    y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Point {
        Point { x, y }
    }
}

#[derive(PartialEq, Eq)]
struct Node {
    point: Point,
    cost: i32,
    h_cost: i32,
    g_cost: i32,
}

impl Node {
    fn new(point: Point, cost: i32, h_cost: i32, g_cost: i32) -> Node {
        Node {
            point,
            cost,
            h_cost,
            g_cost,
        }
    }

    fn create(destination: &Point, point: Point, g_cost: i32) -> Node {
        let h_cost = chebychev(&point, destination);
        Node::new(point, g_cost + h_cost, h_cost, g_cost)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cost.cmp(&other.cost).reverse())
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return if self.cost == other.cost {
            self.h_cost.cmp(&other.h_cost).reverse()
        } else {
            self.cost.cmp(&other.cost).reverse()
        };
    }
}

struct BlockedFlag {
    config: i8,
    dx: i32,
    dy: i32,
}

impl BlockedFlag {
    const fn new(config: i8, dx: i32, dy: i32) -> BlockedFlag {
        BlockedFlag { config, dx, dy }
    }
}

const N: BlockedFlag = BlockedFlag::new(1, 0, 1);
const S: BlockedFlag = BlockedFlag::new(1 << 1, 0, -1);
const E: BlockedFlag = BlockedFlag::new(1 << 2, 1, 0);
const W: BlockedFlag = BlockedFlag::new(1 << 3, -1, 0);
const NE: BlockedFlag = BlockedFlag::new(1 << 4, 1, 1);
const NW: BlockedFlag = BlockedFlag::new(1 << 5, -1, 1);
const SE: BlockedFlag = BlockedFlag::new(1 << 6, 1, -1);
const SW: BlockedFlag = BlockedFlag::new(1 << 7, -1, -1);

const BLOCKED_FLAGS: [BlockedFlag; 8] = [NE, NW, SE, SW, N, S, E, W];

fn chebychev(from: &Point, to: &Point) -> i32 {
    let dx = (from.x - to.x).abs();
    let dy = (from.y - to.y).abs();
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

        let path = pathfinding_grid.find_path(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 8);
    }

    #[test]
    fn test_wall() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = NE.config;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 9);

        let path = pathfinding_grid.find_path(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 9);
    }

    #[test]
    fn test_wall2() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = NE.config;

        for i in 2..8 {
            grid[i][5] = NE.config | N.config | NW.config;
        }

        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 9);

        let path = pathfinding_grid.find_path(&origin, &destination);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 11);
    }

    #[test]
    fn test_no_path() {
        let mut grid = vec![vec![0; 10]; 10];
        grid[1][1] = N.config
            | S.config
            | E.config
            | W.config
            | NE.config
            | NW.config
            | SE.config
            | SW.config;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let origin = Point::new(1, 1);
        let destination = Point::new(9, 3);

        let path = pathfinding_grid.find_path(&origin, &destination);

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
