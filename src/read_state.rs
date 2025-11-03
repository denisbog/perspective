use ::image::ImageReader;
use iced::Size;
use nalgebra::Vector3;
use std::fmt::Debug;
use std::path::Path;
use tracing::warn;

use anyhow::Result;

use crate::AxisData;
use crate::compute::read_points_from_file;
#[derive(Debug, Clone)]
pub struct ImageData {
    pub axis_data: AxisData,
    pub lines: Option<Vec<Vector3<f32>>>,
}
pub async fn load(
    image: String,
    points_file_name: String,
    load_lines: bool,
) -> Result<(Option<ImageData>, Size<u32>)> {
    let extracted_data = if Path::new(&points_file_name).exists() {
        let read_from_file = read_points_from_file(&points_file_name)?;
        let lines = if load_lines { read_from_file.1 } else { None };
        Some(ImageData {
            axis_data: read_from_file.0,
            lines,
        })
    } else {
        warn!("could not read data for {}", points_file_name);
        None
    };

    let decoded_image = ImageReader::open(&image)?.decode()?;
    Ok((
        extracted_data,
        Size::new(decoded_image.width(), decoded_image.height()),
    ))
}
