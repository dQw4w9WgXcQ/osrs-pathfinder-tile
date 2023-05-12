use std::collections::{HashMap, VecDeque};

#[derive(Hash, Eq, PartialEq, Debug, Clone, Copy)]
pub struct Point {
    x: i32,
    y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Point {
        Point { x, y }
    }
}

pub struct PathfindingGrid {
    grid: Vec<Vec<i32>>,
}

impl PathfindingGrid {
    pub fn find_path(&self, from: Point, to: Point) -> Option<Vec<Point>> {
        let mut seen_from: HashMap<Point, Point> = HashMap::new();
        let mut frontier: VecDeque<Point> = VecDeque::new();

        frontier.push_back(from);
        seen_from.insert(from, from);

        let x_size = self.grid.len() as i32;
        let y_size = self.grid[0].len() as i32;

        while !frontier.is_empty() {
            let popped = frontier.pop_front();
            if popped.is_none() {
                return None;
            }

            let current = popped.unwrap();

            if current == to {
                return Some(PathfindingGrid::backtrack(seen_from, current));
            }

            let config = self.grid[current.x as usize][current.y as usize];

            for dir in DIRECTIONS {
                if config & dir.mask != 0 {
                    let x = current.x + dir.dx;
                    let y = current.y + dir.dy;
                    if x < 0 || x >= x_size || y < 0 || y >= y_size {
                        continue;
                    }

                    let next = Point::new(x, y);

                    if !seen_from.contains_key(&next) {
                        frontier.push_back(next);
                        seen_from.insert(next, current);
                    }
                }
            }
        }

        return None;
    }

    fn backtrack(seen_from: HashMap<Point, Point>, end: Point) -> Vec<Point> {
        let mut path: Vec<Point> = Vec::new();
        let mut current = end;
        while current != seen_from[&current] {
            path.push(current);
            current = seen_from[&current];
        }
        path.push(current);
        path.reverse();
        path
    }
}

struct Direction {
    mask: i32,
    dx: i32,
    dy: i32,
}

impl Direction {
    const fn new(mask: i32, dx: i32, dy: i32) -> Direction {
        Direction { mask, dx, dy }
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

fn chebychev(from: Point, to: Point) -> i32 {
    let x = (from.x - to.x).abs();
    let y = (from.y - to.y).abs();
    if x > y {
        x
    } else {
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pathfind() {
        let grid = vec![vec![std::i32::MAX; 10]; 10];

        let pathfinding_grid = PathfindingGrid { grid };

        let from = Point { x: 1, y: 1 };

        let to = Point { x: 9, y: 3 };

        let path: Option<Vec<Point>> = pathfinding_grid.find_path(from, to);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 9);
    }

    #[test]
    fn test_chebychev() {
        print!("idk");
        let from = Point { x: 0, y: 0 };
        let to = Point { x: 3, y: 4 };
        let result = chebychev(from, to);
        assert_eq!(result, 4);

        let from = Point { x: 0, y: 0 };
        let to = Point { x: 3, y: -4 };
        let result = chebychev(from, to);
        assert_eq!(result, 4);
    }
}
