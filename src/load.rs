use std::fs::File;
use std::io::{Cursor, Read};

use array_init::array_init;
use byteorder::{BigEndian, ReadBytesExt};
use zip::ZipArchive;

pub fn load_grid(file_path: &str) -> Result<[Vec<Vec<u8>>; crate::PLANES_SIZE], std::io::Error> {
    let file = File::open(file_path)?;
    let mut archive = ZipArchive::new(file)?;

    let mut grid_file = archive.by_name("grid.dat")?;
    let mut buffer = Vec::new();
    grid_file.read_to_end(&mut buffer)?;

    let mut cursor = Cursor::new(buffer);

    let width = cursor.read_i32::<BigEndian>()? as usize;
    let height = cursor.read_i32::<BigEndian>()? as usize;

    let mut grid = array_init(|_| vec![vec![0u8; height]; width]);

    for plane in &mut grid {
        for col in plane {
            cursor.read_exact(col)?;
        }
    }

    Ok(grid)
}
