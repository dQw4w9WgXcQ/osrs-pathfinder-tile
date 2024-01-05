use osrs_pathfinder_tile::{load_grid, Point, TilePathfinding};

fn main() {
    let grid_planes = load_grid("grid.zip").unwrap();

    let tile_pathfinding = TilePathfinding::new(grid_planes);

    let path_result = tile_pathfinding
        .get_plane(0)
        .find_path(&Point::new(3200, 3200), &Point::new(3222, 3222));

    println!("{:?}", path_result);
}
