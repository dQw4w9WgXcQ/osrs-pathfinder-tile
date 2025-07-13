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
// OSRS-specific constants: diagonal movement has same cost as straight movement
const MOVEMENT_COST: i32 = 10000; // All movements (diagonal, horizontal, vertical) have same cost in OSRS
const HEURISTIC_WEIGHT: i32 = 10000; // Reduced from 100_000 to prevent overflow

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

#[derive(Debug, Display, Deserialize)]
pub enum Algo {
    #[serde(rename = "A_STAR")]
    AStar,
    #[serde(rename = "BFS")]
    Bfs,
    #[serde(rename = "JPS")]
    Jps, // Jump Point Search
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
            Algo::AStar => self.astar_optimized(start, end),
            Algo::Bfs => self.bfs(start, end),
            Algo::Jps => self.jump_point_search(start, end),
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

    /// Optimized A* implementation with better efficiency
    fn astar_optimized(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
        if start == end {
            return Some(vec![*start]);
        }

        // Use Vec with sorting instead of BinaryHeap for better performance on small sets
        let mut open_set = Vec::new();
        let mut closed_set = HashSet::new();
        
        // Combined storage for g_costs and parent tracking
        let mut node_data = HashMap::new();
        
        // Initialize start node
        let start_h = heuristic_optimized(start, end);
        let start_node = AStarNodeOptimized::new(*start, start_h, 0, start_h);
        open_set.push(start_node);
        node_data.insert(*start, NodeData::new(0, None));

        while !open_set.is_empty() {
            // Find node with lowest f_cost (manual min-heap behavior)
            let current_idx = open_set.iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.cmp(b))
                .map(|(idx, _)| idx)
                .unwrap();
            
            let current = open_set.swap_remove(current_idx);

            // Early termination
            if current.point == *end {
                return Some(self.reconstruct_path_optimized(&node_data, *start, *end));
            }

            closed_set.insert(current.point);

            // Explore neighbors
            self.explore_neighbors_optimized(
                &current, 
                end, 
                &mut open_set, 
                &closed_set, 
                &mut node_data
            );
        }

        None
    }

    fn explore_neighbors_optimized(
        &self,
        current: &AStarNodeOptimized,
        end: &Point,
        open_set: &mut Vec<AStarNodeOptimized>,
        closed_set: &HashSet<Point>,
        node_data: &mut HashMap<Point, NodeData>,
    ) {
        let x = current.point.x as usize;
        let y = current.point.y as usize;
        let config = *unsafe { self.grid.get_unchecked(x).get_unchecked(y) };

        for dir in DIRECTIONS {
            if config & dir.flag == 0 {
                continue;
            }

            let neighbor = Point::new(current.point.x + dir.dx, current.point.y + dir.dy);

            if closed_set.contains(&neighbor) {
                continue;
            }

            // In OSRS, all movements (diagonal, horizontal, vertical) have the same cost
            let tentative_g = current.g_cost + MOVEMENT_COST;

            let should_update = match node_data.get(&neighbor) {
                Some(existing) => tentative_g < existing.g_cost,
                None => true,
            };

            if should_update {
                let h_cost = heuristic_optimized(&neighbor, end);
                let f_cost = tentative_g + h_cost;
                
                // Update node data
                node_data.insert(neighbor, NodeData::new(tentative_g, Some(current.point)));
                
                // Remove old entry from open set if it exists
                open_set.retain(|node| node.point != neighbor);
                
                // Add new entry
                open_set.push(AStarNodeOptimized::new(neighbor, f_cost, tentative_g, h_cost));
            }
        }
    }

    fn reconstruct_path_optimized(&self, node_data: &HashMap<Point, NodeData>, start: Point, end: Point) -> Vec<Point> {
        let mut path = Vec::new();
        let mut current = end;
        
        while current != start {
            path.push(current);
            current = node_data.get(&current).unwrap().parent.unwrap();
        }
        path.push(start);
        path.reverse();
        path
    }

    // Keep the original A* for backward compatibility
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
                // In OSRS, all movements have the same cost regardless of direction
                let next_g_cost = curr.g_cost + MOVEMENT_COST;

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
                path.push(*start);
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

    /// Jump Point Search (JPS) - Advanced optimization for grid-based pathfinding
    /// Reduces the number of nodes explored by "jumping" over intermediate nodes
    fn jump_point_search(&self, start: &Point, end: &Point) -> Option<Vec<Point>> {
        if start == end {
            return Some(vec![*start]);
        }

        let mut open_set = Vec::new();
        let mut closed_set = HashSet::new();
        let mut node_data = HashMap::new();

        let start_h = heuristic_optimized(start, end);
        let start_node = AStarNodeOptimized::new(*start, start_h, 0, start_h);
        open_set.push(start_node);
        node_data.insert(*start, NodeData::new(0, None));

        while !open_set.is_empty() {
            let current_idx = open_set.iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.cmp(b))
                .map(|(idx, _)| idx)
                .unwrap();
            
            let current = open_set.swap_remove(current_idx);

            if current.point == *end {
                return Some(self.reconstruct_path_optimized(&node_data, *start, *end));
            }

            closed_set.insert(current.point);

            // Get the parent to determine the direction we came from
            let parent = node_data.get(&current.point).and_then(|data| data.parent);
            
            // Get forced neighbors and natural neighbors
            let neighbors = self.get_jump_neighbors(&current.point, parent);

            for neighbor in neighbors {
                if closed_set.contains(&neighbor) {
                    continue;
                }

                // Jump to the next jump point
                if let Some(jump_point) = self.jump(&current.point, &neighbor, end) {
                    let distance = self.distance(&current.point, &jump_point);
                    let tentative_g = current.g_cost + distance;

                    let should_update = match node_data.get(&jump_point) {
                        Some(existing) => tentative_g < existing.g_cost,
                        None => true,
                    };

                    if should_update {
                        let h_cost = heuristic_optimized(&jump_point, end);
                        let f_cost = tentative_g + h_cost;

                        node_data.insert(jump_point, NodeData::new(tentative_g, Some(current.point)));
                        open_set.retain(|node| node.point != jump_point);
                        open_set.push(AStarNodeOptimized::new(jump_point, f_cost, tentative_g, h_cost));
                    }
                }
            }
        }

        None
    }

    /// Get jump neighbors based on the current position and parent
    fn get_jump_neighbors(&self, current: &Point, parent: Option<Point>) -> Vec<Point> {
        let mut neighbors = Vec::new();

        if let Some(parent) = parent {
            let dx = (current.x - parent.x).signum();
            let dy = (current.y - parent.y).signum();

            if dx != 0 && dy != 0 {
                // Diagonal movement
                self.add_diagonal_jump_neighbors(current, dx, dy, &mut neighbors);
            } else {
                // Straight movement
                self.add_straight_jump_neighbors(current, dx, dy, &mut neighbors);
            }
        } else {
            // No parent, explore all directions
            for dir in &DIRECTIONS {
                let neighbor = Point::new(current.x + dir.dx, current.y + dir.dy);
                if self.is_walkable(&neighbor) {
                    neighbors.push(neighbor);
                }
            }
        }

        neighbors
    }

    fn add_diagonal_jump_neighbors(&self, current: &Point, dx: i32, dy: i32, neighbors: &mut Vec<Point>) {
        // Straight components
        if self.is_walkable(&Point::new(current.x + dx, current.y)) {
            neighbors.push(Point::new(current.x + dx, current.y));
        }
        if self.is_walkable(&Point::new(current.x, current.y + dy)) {
            neighbors.push(Point::new(current.x, current.y + dy));
        }

        // Diagonal
        if self.is_walkable(&Point::new(current.x + dx, current.y + dy)) {
            neighbors.push(Point::new(current.x + dx, current.y + dy));
        }

        // Forced neighbors
        if !self.is_walkable(&Point::new(current.x - dx, current.y)) && 
           self.is_walkable(&Point::new(current.x - dx, current.y + dy)) {
            neighbors.push(Point::new(current.x - dx, current.y + dy));
        }
        if !self.is_walkable(&Point::new(current.x, current.y - dy)) && 
           self.is_walkable(&Point::new(current.x + dx, current.y - dy)) {
            neighbors.push(Point::new(current.x + dx, current.y - dy));
        }
    }

    fn add_straight_jump_neighbors(&self, current: &Point, dx: i32, dy: i32, neighbors: &mut Vec<Point>) {
        if dx != 0 {
            // Horizontal movement
            if self.is_walkable(&Point::new(current.x + dx, current.y)) {
                neighbors.push(Point::new(current.x + dx, current.y));
            }
            
            // Forced neighbors
            if !self.is_walkable(&Point::new(current.x, current.y + 1)) && 
               self.is_walkable(&Point::new(current.x + dx, current.y + 1)) {
                neighbors.push(Point::new(current.x + dx, current.y + 1));
            }
            if !self.is_walkable(&Point::new(current.x, current.y - 1)) && 
               self.is_walkable(&Point::new(current.x + dx, current.y - 1)) {
                neighbors.push(Point::new(current.x + dx, current.y - 1));
            }
        } else {
            // Vertical movement
            if self.is_walkable(&Point::new(current.x, current.y + dy)) {
                neighbors.push(Point::new(current.x, current.y + dy));
            }
            
            // Forced neighbors
            if !self.is_walkable(&Point::new(current.x + 1, current.y)) && 
               self.is_walkable(&Point::new(current.x + 1, current.y + dy)) {
                neighbors.push(Point::new(current.x + 1, current.y + dy));
            }
            if !self.is_walkable(&Point::new(current.x - 1, current.y)) && 
               self.is_walkable(&Point::new(current.x - 1, current.y + dy)) {
                neighbors.push(Point::new(current.x - 1, current.y + dy));
            }
        }
    }

    /// Jump from current position towards direction until a jump point is found
    fn jump(&self, current: &Point, direction: &Point, goal: &Point) -> Option<Point> {
        let dx = direction.x - current.x;
        let dy = direction.y - current.y;
        let next = Point::new(current.x + dx, current.y + dy);

        if !self.is_walkable(&next) {
            return None;
        }

        if next == *goal {
            return Some(next);
        }

        // Check for forced neighbors
        if self.has_forced_neighbors(&next, dx, dy) {
            return Some(next);
        }

        // Diagonal movement: check horizontal and vertical jumps
        if dx != 0 && dy != 0 {
            if self.jump(&next, &Point::new(next.x + dx, next.y), goal).is_some() ||
               self.jump(&next, &Point::new(next.x, next.y + dy), goal).is_some() {
                return Some(next);
            }
        }

        // Recursively jump in the same direction
        self.jump(&next, direction, goal)
    }

    fn has_forced_neighbors(&self, point: &Point, dx: i32, dy: i32) -> bool {
        if dx != 0 && dy != 0 {
            // Diagonal movement
            (!self.is_walkable(&Point::new(point.x - dx, point.y)) && 
             self.is_walkable(&Point::new(point.x - dx, point.y + dy))) ||
            (!self.is_walkable(&Point::new(point.x, point.y - dy)) && 
             self.is_walkable(&Point::new(point.x + dx, point.y - dy)))
        } else if dx != 0 {
            // Horizontal movement
            (!self.is_walkable(&Point::new(point.x, point.y + 1)) && 
             self.is_walkable(&Point::new(point.x + dx, point.y + 1))) ||
            (!self.is_walkable(&Point::new(point.x, point.y - 1)) && 
             self.is_walkable(&Point::new(point.x + dx, point.y - 1)))
        } else {
            // Vertical movement
            (!self.is_walkable(&Point::new(point.x + 1, point.y)) && 
             self.is_walkable(&Point::new(point.x + 1, point.y + dy))) ||
            (!self.is_walkable(&Point::new(point.x - 1, point.y)) && 
             self.is_walkable(&Point::new(point.x - 1, point.y + dy)))
        }
    }

    fn is_walkable(&self, point: &Point) -> bool {
        if !self.in_bounds(point) {
            return false;
        }
        let x = point.x as usize;
        let y = point.y as usize;
        let config = *unsafe { self.grid.get_unchecked(x).get_unchecked(y) };
        config != 0
    }

    fn distance(&self, from: &Point, to: &Point) -> i32 {
        let dx = (to.x - from.x).abs();
        let dy = (to.y - from.y).abs();
        // In OSRS, all movements have the same cost, so use Chebyshev distance
        let chebyshev_distance = std::cmp::max(dx, dy);
        chebyshev_distance * MOVEMENT_COST
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

#[derive(Clone, PartialEq, Eq, Debug)]
struct AStarNodeOptimized {
    point: Point,
    f_cost: i32,
    g_cost: i32,
    h_cost: i32,
}

impl AStarNodeOptimized {
    fn new(point: Point, f_cost: i32, g_cost: i32, h_cost: i32) -> Self {
        Self { point, f_cost, g_cost, h_cost }
    }
}

impl PartialOrd for AStarNodeOptimized {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNodeOptimized {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Lower f_cost is better
        match self.f_cost.cmp(&other.f_cost) {
            std::cmp::Ordering::Equal => {
                // Tie-breaker: prefer lower h_cost (closer to goal)
                self.h_cost.cmp(&other.h_cost)
            }
            other => other,
        }
    }
}

#[derive(Clone, Debug)]
struct NodeData {
    g_cost: i32,
    parent: Option<Point>,
}

impl NodeData {
    fn new(g_cost: i32, parent: Option<Point>) -> Self {
        Self { g_cost, parent }
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

/// Optimized heuristic function for OSRS (all movements have same cost)
fn heuristic_optimized(a: &Point, b: &Point) -> i32 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    
    // In OSRS, diagonal movement has same cost as straight movement
    // So we use Chebyshev distance (max of dx, dy) as heuristic
    let chebyshev_distance = std::cmp::max(dx, dy);
    
    // Scale by movement cost for proper comparison
    chebyshev_distance * MOVEMENT_COST
}

// Original heuristic function for backward compatibility
fn heuristic(a: &Point, b: &Point) -> i32 {
    let chebyshev = chebyshev(a, b);
    let diagonal_cost = diagonal_cost(a, b);

    (chebyshev * HEURISTIC_WEIGHT) + diagonal_cost
}

fn diagonal_cost(a: &Point, b: &Point) -> i32 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    (dx - dy).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar() {
        let grid = vec![vec![!0; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);

        let path = pathfinding_grid.astar(&start, &end);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 9);
    }

    #[test]
    fn test_astar_wall() {
        let mut grid = vec![vec![255; 10]; 10];
        grid[1][1] &= !NE.flag;

        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 9);

        let path = pathfinding_grid.astar(&start, &end);
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 10);
    }

    #[test]
    fn test_astar_wall2() {
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
        assert_eq!(path.unwrap().len(), 12);
    }

    #[test]
    fn test_astar_no_path() {
        let mut grid = vec![vec![!0; 10]; 10];
        grid[1][1] = 0;
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let end = Point::new(9, 3);

        let path = pathfinding_grid.astar(&start, &end);

        assert!(path.is_none());
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
    fn test_find_distances_start_end_equal() {
        let grid = vec![vec![!0; 10]; 10];
        let pathfinding_grid = PathfindingGrid::new(grid);

        let start = Point::new(1, 1);
        let mut ends = Vec::new();
        ends.push(Point::new(1, 1));

        let distances = pathfinding_grid.find_distances(&start, ends).unwrap();

        assert_eq!(distances.len(), 1);

        let distance = distances.get(0).unwrap().1;
        assert_eq!(distance, 0);
    }

    #[test]
    fn test_chebyshev() {
        let a = chebyshev(&Point::new(0, 0), &Point::new(3, 4));
        assert_eq!(a, 4);
        let b = chebyshev(&Point::new(0, 0), &Point::new(-3, 4));
        assert_eq!(a, b);
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
