use std::fs::File;
use std::io::{self, Cursor, Read};

use byteorder::{BigEndian, ReadBytesExt};
use zip::read::ZipArchive;

use crate::PLANES_SIZE;

pub fn load_grid(file_path: &str) -> io::Result<[Vec<Vec<u8>>; PLANES_SIZE]> {
    let file = File::open(file_path)?;
    let mut archive = ZipArchive::new(file)?;
    let mut grid_file = archive.by_name("grid.dat")?;
    let mut buffer = Vec::new();
    grid_file.read_to_end(&mut buffer)?;
    let mut cursor = Cursor::new(buffer);

    let width = cursor.read_i32::<BigEndian>()? as usize;
    let height = cursor.read_i32::<BigEndian>()? as usize;

    let mut grid = [vec![Vec::with_capacity(width); height]; PLANES_SIZE];
    for plane in &mut grid {
        for col in plane {
            cursor.read_exact(col)?;
        }
    }

    Ok(grid)
}
